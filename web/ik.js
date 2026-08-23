// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Pose-based control for OpenArm on the MuJoCo WASM bindings.
//
// PoseController owns a kinematics-only MjData used as an IK scratch space:
// solve() runs damped-least-squares IK there and returns exact joint targets
// for a requested end-effector pose, without touching the simulation state.

function mat2quat(m) {
  const q = [0, 0, 0, 0];
  const tr = m[0] + m[4] + m[8];
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q[0] = 0.25 * s;
    q[1] = (m[7] - m[5]) / s;
    q[2] = (m[2] - m[6]) / s;
    q[3] = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
    q[0] = (m[7] - m[5]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[1] + m[3]) / s;
    q[3] = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
    q[0] = (m[2] - m[6]) / s;
    q[1] = (m[1] + m[3]) / s;
    q[2] = 0.25 * s;
    q[3] = (m[5] + m[7]) / s;
  } else {
    const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
    q[0] = (m[3] - m[1]) / s;
    q[1] = (m[2] + m[6]) / s;
    q[2] = (m[5] + m[7]) / s;
    q[3] = 0.25 * s;
  }
  return q;
}

export function quatMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function quatConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

function quatRotVec(q, v) {
  const [w, x, y, z] = q;
  const [vx, vy, vz] = v;
  // t = 2 q_vec x v; v' = v + w t + q_vec x t
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + y * tz - z * ty,
    vy + w * ty + z * tx - x * tz,
    vz + w * tz + x * ty - y * tx,
  ];
}

// Express a pose given in a local frame (e.g. the arm_origin site) in world
// coordinates.
export function poseLocalToWorld(origin, pos, quat) {
  const p = quatRotVec(origin.quat, pos);
  return {
    pos: [origin.pos[0] + p[0], origin.pos[1] + p[1], origin.pos[2] + p[2]],
    quat: quatMul(origin.quat, quat),
  };
}

// Inverse of poseLocalToWorld.
function poseWorldToLocal(origin, pos, quat) {
  const inv = quatConj(origin.quat);
  return {
    pos: quatRotVec(inv, [
      pos[0] - origin.pos[0],
      pos[1] - origin.pos[1],
      pos[2] - origin.pos[2],
    ]),
    quat: quatMul(inv, quat),
  };
}

export function eulerZYXToQuat(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2),
    sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2),
    sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2),
    sy = Math.sin(yaw / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

// Rotation error as a world-frame rotation vector: log(q_target * q_current^-1)
export function quatError(qTarget, qCurrent) {
  let qe = quatMul(qTarget, quatConj(qCurrent));
  if (qe[0] < 0) qe = qe.map((v) => -v);
  const sin = Math.hypot(qe[1], qe[2], qe[3]);
  if (sin < 1e-10) return [0, 0, 0];
  const angle = 2 * Math.atan2(sin, qe[0]);
  return [(qe[1] / sin) * angle, (qe[2] / sin) * angle, (qe[3] / sin) * angle];
}

// Solve (A + lambda2 I) x = b for symmetric 6x6 A, Gaussian elimination.
function solve6(A, b, lambda2) {
  const n = 6;
  const M = A.map((row, i) => {
    const r = [...row];
    r[i] += lambda2;
    r.push(b[i]);
    return r;
  });
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++)
      if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c || M[c][c] === 0) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / M[i][i]);
}

class Arm {
  constructor(mujoco, model, side) {
    this.side = side;
    this.dofIds = [];
    this.qposIds = [];
    this.jntRanges = [];
    for (let i = 1; i <= 7; i++) {
      const j = model.jnt(`openarm_${side}_joint${i}`);
      this.dofIds.push(Number(j.dofadr[0] ?? j.dofadr));
      this.qposIds.push(Number(j.qposadr[0] ?? j.qposadr));
      this.jntRanges.push([j.range[0], j.range[1]]);
      j.delete();
    }
    this.actIds = [];
    for (let i = 1; i <= 7; i++) {
      const a = model.actuator(`${side}_joint${i}_ctrl`);
      this.actIds.push(a.id);
      a.delete();
    }
    const g = model.actuator(`${side}_finger1_ctrl`);
    this.gripperActId = g.id;
    this.gripperRange = [g.ctrlrange[0], g.ctrlrange[1]];
    g.delete();
    this.siteId = mujoco.mj_name2id(
      model,
      mujoco.mjtObj.mjOBJ_SITE.value,
      `${side}_ee_control_point`,
    );
  }

  pose(data) {
    const p = data.site_xpos.subarray(this.siteId * 3, this.siteId * 3 + 3);
    const m = data.site_xmat.subarray(this.siteId * 9, this.siteId * 9 + 9);
    return { pos: [...p], quat: mat2quat(m) };
  }

  poseError(data, targetPos, targetQuat) {
    const { pos, quat } = this.pose(data);
    return {
      errorP: [
        targetPos[0] - pos[0],
        targetPos[1] - pos[1],
        targetPos[2] - pos[2],
      ],
      errorR: quatError(targetQuat, quat),
    };
  }
}

export class PoseController {
  constructor(mujoco, model) {
    this.mujoco = mujoco;
    this.model = model;
    this.nv = model.nv;
    this.ikData = new mujoco.MjData(model);
    this.jacp = new mujoco.DoubleBuffer(3 * this.nv);
    this.jacr = new mujoco.DoubleBuffer(3 * this.nv);
    try {
      this.arms = {
        left: new Arm(mujoco, model, "left"),
        right: new Arm(mujoco, model, "right"),
      };
      // Pose targets are expressed in the arm_origin site frame; -1 (absent)
      // means the model root is the origin and targets are world coordinates.
      this.originSiteId = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE.value,
        "arm_origin",
      );
      // Optional lifter (cell scenes): the arms ride on a vertical slide
      // joint.
      this.lifterDofId = -1;
      this.lifterActId = -1;
      const lifterJointId = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT.value,
        "openarm_lifter_joint",
      );
      this.lifterQposId = -1;
      if (lifterJointId >= 0) {
        const j = model.jnt(lifterJointId);
        this.lifterDofId = Number(j.dofadr[0] ?? j.dofadr);
        this.lifterQposId = Number(j.qposadr[0] ?? j.qposadr);
        j.delete();
        const a = model.actuator("lifter_ctrl");
        this.lifterActId = a.id;
        a.delete();
      }
    } catch (e) {
      // e.g. a model without the openarm_{side}_joint1..7 naming: free the
      // buffers allocated above instead of leaking them in the WASM heap.
      this.dispose();
      throw e;
    }
  }

  // World pose of the arm_origin site (identity if the model has none).
  originPose(data) {
    if (this.originSiteId < 0) return { pos: [0, 0, 0], quat: [1, 0, 0, 0] };
    const p = data.site_xpos.subarray(
      this.originSiteId * 3,
      this.originSiteId * 3 + 3,
    );
    const m = data.site_xmat.subarray(
      this.originSiteId * 9,
      this.originSiteId * 9 + 9,
    );
    return { pos: [...p], quat: mat2quat(m) };
  }

  dispose() {
    this.jacr.delete();
    this.jacp.delete();
    this.ikData.delete();
  }

  // Seed the IK scratch state from the simulation state.
  syncFrom(data) {
    this.ikData.qpos.set(data.qpos);
    this.lastCommands = {}; // the IK seed changed; cached solutions are stale
  }

  // Damped-least-squares IK on the scratch state; returns 7 joint targets.
  solve(
    side,
    targetPos,
    targetQuat,
    { iters = 50, tol = 1e-5, maxStep = 0.2, lambda2 = 1e-6 } = {},
  ) {
    const { mujoco, model, ikData, nv } = this;
    const arm = this.arms[side];
    const qpos = ikData.qpos;
    for (let it = 0; it < iters; it++) {
      mujoco.mj_kinematics(model, ikData);
      mujoco.mj_comPos(model, ikData);
      const { errorP, errorR } = arm.poseError(ikData, targetPos, targetQuat);
      if (Math.hypot(...errorP) < tol && Math.hypot(...errorR) < 10 * tol)
        break;
      const error = [...errorP, ...errorR];

      mujoco.mj_jacSite(model, ikData, this.jacp, this.jacr, arm.siteId);
      const Jp = this.jacp.GetView();
      const Jr = this.jacr.GetView();
      const J = [];
      for (let r = 0; r < 3; r++) J.push(arm.dofIds.map((d) => Jp[r * nv + d]));
      for (let r = 0; r < 3; r++) J.push(arm.dofIds.map((d) => Jr[r * nv + d]));
      const A = [];
      for (let i = 0; i < 6; i++) {
        A.push([]);
        for (let j = 0; j < 6; j++) {
          let s = 0;
          for (let k = 0; k < 7; k++) s += J[i][k] * J[j][k];
          A[i].push(s);
        }
      }
      const y = solve6(A, error, lambda2);
      for (let k = 0; k < 7; k++) {
        let s = 0;
        for (let i = 0; i < 6; i++) s += J[i][k] * y[i];
        s = Math.min(maxStep, Math.max(-maxStep, s));
        const [lo, hi] = arm.jntRanges[k];
        qpos[arm.qposIds[k]] = Math.min(
          hi,
          Math.max(lo, qpos[arm.qposIds[k]] + s),
        );
      }
    }
    return arm.qposIds.map((i) => qpos[i]);
  }

  // Solve IK for a pose target and write the joint targets to data.ctrl.
  command(data, side, targetPos, targetQuat) {
    // keep the IK scratch state's lifter in step with the simulation, so a
    // world target that already includes the lifted arm_origin is not reached
    // a second time by stretching the arm
    if (this.lifterQposId >= 0) {
      this.ikData.qpos[this.lifterQposId] = data.qpos[this.lifterQposId];
    }
    const arm = this.arms[side];
    const q = this.solve(side, targetPos, targetQuat);
    q.forEach((v, i) => {
      data.ctrl[arm.actIds[i]] = v;
    });
    return q;
  }

  commandGripper(data, side, openRatio) {
    const arm = this.arms[side];
    const [lo, hi] = arm.gripperRange;
    // For both arms ratio 0 = closed (ctrl 0), 1 = fully open (range end away from 0)
    const end = Math.abs(lo) > Math.abs(hi) ? lo : hi;
    data.ctrl[arm.gripperActId] = end * openRatio;
  }

  commandLifter(data, height) {
    if (this.lifterActId >= 0) data.ctrl[this.lifterActId] = height;
  }

  // Reset to the scene's first keyframe, with the position actuators holding
  // that pose (ctrl = qpos): without it they would drive every joint toward
  // 0 rad on the first step.
  startFromKeyframe(data) {
    const { mujoco, model } = this;
    mujoco.mj_resetDataKeyframe(model, data, 0);
    mujoco.mj_forward(model, data);
    for (const side of ["left", "right"]) {
      const arm = this.arms[side];
      for (let i = 0; i < 7; i++) {
        data.ctrl[arm.actIds[i]] = data.qpos[arm.qposIds[i]];
      }
    }
  }

  // Reset to the IK solution of teleop's home pose, with the position
  // actuators holding it — for scenes that define no keyframe.
  startFromTeleopHome(data, teleop) {
    const { mujoco, model } = this;
    mujoco.mj_resetData(model, data);
    mujoco.mj_forward(model, data);
    this.syncFrom(data);
    const origin = this.originPose(data);
    for (const side of ["left", "right"]) {
      const arm = teleop.arms[side];
      const { pos, quat } = poseLocalToWorld(origin, arm.pos, arm.quat);
      this.solve(side, pos, quat, { iters: 300 });
    }
    data.qpos.set(this.ikData.qpos);
    data.qvel.fill(0);
    mujoco.mj_forward(model, data);
    for (const side of ["left", "right"]) {
      const arm = this.arms[side];
      for (let i = 0; i < 7; i++) {
        data.ctrl[arm.actIds[i]] = data.qpos[arm.qposIds[i]];
      }
    }
  }

  // Reset to the scene's start pose: the `home` keyframe when the model has
  // one (its end-effector pose in the arm_origin frame is the teleop home
  // pose, same contract as dora-openarm-keyboard), otherwise the IK solution
  // of teleop's home pose. The measured start pose becomes teleop's home, so
  // the first tick never yanks and a teleop reset returns here.
  startFromHome(data, teleop) {
    if (this.model.nkey > 0) this.startFromKeyframe(data);
    else this.startFromTeleopHome(data, teleop);
    this.syncFrom(data);
    const origin = this.originPose(data);
    for (const side of ["left", "right"]) {
      const { pos, quat } = this.arms[side].pose(data);
      const local = poseWorldToLocal(origin, pos, quat);
      teleop.arms[side].setHome(local.pos, local.quat);
    }
  }

  // Send teleop's current targets (expressed in the arm_origin frame) to the
  // actuators. Returns each side's world target for callers that need it.
  //
  // The joint solution depends only on the arm_origin-frame target (the arm
  // rides on the lifter together with its target), so an unchanged target
  // reuses the previous solution. Without this, an out-of-reach target —
  // teleop clamps positions at ±0.8 m, beyond the arm's reach — would never
  // converge and would cost the full IK iteration budget for both arms on
  // every frame, forever.
  applyTeleop(data, teleop, lifterHeight = 0) {
    this.commandLifter(data, lifterHeight);
    const origin = this.originPose(data);
    const targets = {};
    this.lastCommands ??= {};
    for (const side of ["left", "right"]) {
      const arm = teleop.arms[side];
      const target = poseLocalToWorld(origin, arm.pos, arm.quat);
      const last = this.lastCommands[side];
      if (
        last?.pos.every((v, i) => v === arm.pos[i]) &&
        last.quat.every((v, i) => v === arm.quat[i])
      ) {
        last.q.forEach((v, i) => {
          data.ctrl[this.arms[side].actIds[i]] = v;
        });
      } else {
        const q = this.command(data, side, target.pos, target.quat);
        this.lastCommands[side] = { pos: [...arm.pos], quat: [...arm.quat], q };
      }
      this.commandGripper(data, side, 1 - arm.grip);
      targets[side] = target;
    }
    return targets;
  }

  // Gravity/bias compensation so the low-gain position actuators do not sag.
  applyGravityComp(data) {
    for (const side of ["left", "right"]) {
      for (const d of this.arms[side].dofIds)
        data.qfrc_applied[d] = data.qfrc_bias[d];
    }
    if (this.lifterDofId >= 0) {
      data.qfrc_applied[this.lifterDofId] = data.qfrc_bias[this.lifterDofId];
    }
  }
}
