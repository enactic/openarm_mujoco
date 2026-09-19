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

// WebXR controller poses to arm_origin-frame pose targets, ported from
// dora-openarm-webxr (src/dora_openarm_webxr/main.py and smoothing.py).
//
// There the browser sends every WebXR frame to a Python node that converts
// the controller pose into the OpenArm workspace, smooths it with a One Euro
// filter and publishes it for the IK downstream. Here the page is the whole
// pipeline, so that conversion runs in the browser and feeds TeleopState
// (teleop.js) directly: XRTeleop.processFrame() takes the same frame object
// the dora client sends (see xr-frame.js) and overwrites the teleop targets
// with the controller poses.
//
// Quaternions are [w, x, y, z] like the rest of this app (ik.js); WebXR's
// {x, y, z, w} orientation is converted on the way in.
import { calibratePivot, PivotCalibration } from "./calibration.js";
import { mat2quat, quatConj, quatMul, quatRotVec } from "./ik.js";
import { LEFT, RIGHT } from "./keymap.js";

// Relative pose to robot workspace mapping (main.py: _ROBOT_ROTATION). WebXR
// is x right, y up, -z forward; the robot is x forward, y left, z up.
export const ROBOT_ROTATION = mat2quat([0, 0, -1, -1, 0, 0, 0, 1, 0]);

// Neutral hand position relative to the arm_origin site (chest level):
// main.py's _FRAME_OFFSET_CELL.
export const DEFAULT_FRAME_OFFSET = [-0.085, 0, -0.14];

// The controller is read from the WebXR target ray space, the OpenXR aim
// pose whose -Z points where the controller points. This turn maps that aim
// frame onto the end effector one, which points the gripper along its own -z
// and opens it across y (main.py: _CONTROLLER_TO_EE, z +90 degrees).
export const CONTROLLER_TO_EE = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];

// Eyes to the neck's rotation axis, in the headset's own frame (-Z forward,
// +Y up: below and behind the face). The head turns about the neck, not the
// headset, so subtracting the headset position alone would read every head
// turn as the operator translating and drag the target along the arc the
// headset travels. An estimate that anatomy varies: measure it with the
// neck pivot calibration (calibration.js). [0, 0, 0] is the plain headset
// subtraction (main.py: _NECK_PIVOT_OFFSET).
export const DEFAULT_NECK_PIVOT_OFFSET = [0.0, -0.075, 0.08];

// Where the operator's eyes go relative to the arm_origin site, in its
// frame (x forward, z up), when the world is placed for a headset. OpenArm
// has no head, so this is a human one: the eyes sit some 22 cm above the
// shoulder joints and a little ahead of them.
export const HEAD_OFFSET = [0.03, 0, 0.22];

// One Euro filter parameters main.py builds its per-hand smoothers with.
const SMOOTHER_OPTIONS = { minCutoff: 2.0, beta: 0.04, dCutoff: 1.5 };

export const SIDES = [LEFT, RIGHT];

// WebXR {x, y, z, qx, qy, qz, qw} -> position and [w, x, y, z] quaternion.
export function xrPose(p) {
  return { pos: [p.x, p.y, p.z], quat: [p.qw, p.qx, p.qy, p.qz] };
}

function slerp(q1, q2, alpha) {
  let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  let b = q2;
  if (dot < 0) {
    b = q2.map((v) => -v);
    dot = -dot;
  }
  if (dot > 0.9995) {
    const res = q1.map((v, i) => v + alpha * (b[i] - v));
    const n = Math.hypot(...res);
    return res.map((v) => v / n);
  }
  const theta0 = Math.acos(dot);
  const sinTheta0 = Math.sin(theta0);
  const theta = theta0 * alpha;
  const sinTheta = Math.sin(theta);
  const s0 = Math.cos(theta) - (dot * sinTheta) / sinTheta0;
  const s1 = sinTheta / sinTheta0;
  return q1.map((v, i) => s0 * v + s1 * b[i]);
}

// One Euro filter applied to position (adaptive cutoff) and rotation (slerp
// with the same alpha); port of smoothing.py's OneEuroPoseSmoother. Poses
// are 7-vectors [x, y, z, qw, qx, qy, qz]; time is in seconds.
export class OneEuroPoseSmoother {
  constructor({ minCutoff = 10.0, beta = 0.8, dCutoff = 1.0 } = {}) {
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.dCutoff = dCutoff;
    this.reset();
  }

  // Forget the history: the next sample starts the filter afresh.
  reset() {
    this.pPrev = null;
    this.qPrev = null;
    this.dpPrev = [0, 0, 0];
    this.tPrev = null;
  }

  smooth(t, pose) {
    const tp = pose.slice(0, 3);
    const tq = pose.slice(3, 7);
    if (this.tPrev === null || this.pPrev === null) {
      this.pPrev = tp;
      this.qPrev = tq;
      this.tPrev = t;
      return [...pose];
    }
    const dt = t - this.tPrev;
    if (dt <= 0) return [...pose];

    const alpha = (cutoff) => {
      const tau = 1 / (2 * Math.PI * cutoff);
      return 1 / (1 + tau / dt);
    };
    const dpRaw = tp.map((v, i) => (v - this.pPrev[i]) / dt);
    const alphaD = alpha(this.dCutoff);
    const dp = dpRaw.map((v, i) => alphaD * v + (1 - alphaD) * this.dpPrev[i]);
    const cutoffP = this.minCutoff + this.beta * Math.hypot(...dp);
    const alphaP = alpha(cutoffP);
    const pHat = this.pPrev.map((v, i) => v + alphaP * (tp[i] - v));
    const qHat = slerp(this.qPrev, tq, alphaP);

    this.pPrev = pHat;
    this.qPrev = qHat;
    this.dpPrev = dp;
    this.tPrev = t;
    return [...pHat, ...qHat];
  }
}

// The neck pivot in WebXR coordinates for a headset pose: behind the face,
// wherever the head is facing.
export function neckPivot(reference, neckPivotOffset) {
  const ref = xrPose(reference);
  const offset = quatRotVec(ref.quat, neckPivotOffset);
  return ref.pos.map((v, i) => v + offset[i]);
}

// Convert a WebXR controller pose into an arm_origin-frame pose target
// (main.py: _adjust_pose, minus the smoothing).
//
// `pose` and `reference` (the viewer pose) are in the same world-fixed
// reference space. Only the position is made relative to the viewer, by
// subtracting the neck pivot in the world axes. The viewer rotation is never
// applied to the hand: turning the head must not move the target. It only
// places the pivot, which says which way the operator is facing.
export function adjustPose(pose, reference, { frameOffset, neckPivotOffset }) {
  const pivot = neckPivot(reference, neckPivotOffset);
  const hand = xrPose(pose);
  const relative = hand.pos.map((v, i) => v - pivot[i]);
  const rotated = quatRotVec(ROBOT_ROTATION, relative);
  return {
    pos: rotated.map((v, i) => v + frameOffset[i]),
    quat: quatMul(ROBOT_ROTATION, quatMul(hand.quat, CONTROLLER_TO_EE)),
  };
}

// Inverse of adjustPose for positions: where in WebXR coordinates a hand has
// to be for its target to land on `pos` in the arm_origin frame.
export function robotToXR(pos, reference, { frameOffset, neckPivotOffset }) {
  const pivot = neckPivot(reference, neckPivotOffset);
  const relative = quatRotVec(
    quatConj(ROBOT_ROTATION),
    pos.map((v, i) => v - frameOffset[i]),
  );
  return relative.map((v, i) => v + pivot[i]);
}

// Full inverse of adjustPose: the WebXR pose object ({x, y, z, qx, qy, qz,
// qw}) of a hand whose target is `pos`, `quat` in the arm_origin frame.
export function robotPoseToXR(pos, quat, reference, config) {
  const [x, y, z] = robotToXR(pos, reference, config);
  const [qw, qx, qy, qz] = quatMul(
    quatConj(ROBOT_ROTATION),
    quatMul(quat, quatConj(CONTROLLER_TO_EE)),
  );
  return { x, y, z, qx, qy, qz, qw };
}

// Where to draw the MuJoCo world in the WebXR reference space: the rigid
// transform that takes MuJoCo world coordinates to WebXR coordinates, given
// the world pose of the arm_origin site and the headset pose the placement
// is anchored to.
//
// The rotation turns the arm_origin frame the way adjustPose turns the
// controllers (R^T q_o^-1): z-up becomes y-up and the robot's +x is ahead
// of the operator. The translation is fixed by one anchor point:
//
// * By default the arm targets coincide with the controllers. Solving
//   adjustPose for the world, a point r in the arm_origin frame is at
//   R^T (r - f) + pivot in WebXR, so the arm_origin site itself goes to
//   R^T (-f) + pivot.
// * With `anchor` ({ world, xr }), the MuJoCo world point `world` is drawn
//   at the WebXR point `xr` instead: e.g. headAnchor, which puts the
//   headset where the robot's head would be.
export function worldPlacement(origin, reference, config, anchor = null) {
  const quat = quatMul(quatConj(ROBOT_ROTATION), quatConj(origin.quat));
  const world = anchor ? anchor.world : origin.pos;
  const xr = anchor ? anchor.xr : robotToXR([0, 0, 0], reference, config);
  const shifted = quatRotVec(quat, world);
  return { pos: xr.map((v, i) => v - shifted[i]), quat };
}

// The anchor that draws the robot's head position (HEAD_OFFSET from the
// arm_origin site) at the headset: the operator looks out from where the
// robot's head would be, with the arms below them like their own.
export function headAnchor(origin, reference, headOffset = HEAD_OFFSET) {
  const offset = quatRotVec(origin.quat, headOffset);
  return {
    world: origin.pos.map((v, i) => v + offset[i]),
    xr: [reference.x, reference.y, reference.z],
  };
}

// Per-session state (main.py: _ConnectionState + _process_frame): the
// smoothers and the calibration are stateful, so a new session gets a fresh
// one rather than inheriting the last one's history.
export class XRTeleop {
  constructor({
    frameOffset = DEFAULT_FRAME_OFFSET,
    neckPivotOffset = DEFAULT_NECK_PIVOT_OFFSET,
    calibration = false,
    onCalibrationResult = null,
  } = {}) {
    this.frameOffset = [...frameOffset];
    this.neckPivotOffset = [...neckPivotOffset];
    this.onCalibrationResult = onCalibrationResult;
    this.calibration = new PivotCalibration({ enabled: calibration });
    this.smoothers = {};
    for (const side of SIDES) {
      this.smoothers[side] = new OneEuroPoseSmoother(SMOOTHER_OPTIONS);
    }
  }

  // Take one frame (the object xr-frame.js's readFrame builds) at `time`
  // seconds and write the controller poses into `teleop`'s arm targets.
  // Returns which sides were updated.
  processFrame(response, time, teleop) {
    // An absent button is a released one, so a controller that falls asleep
    // mid-run cannot leave the hands stopped.
    const samples = this.calibration.update(response.button_y === true);
    if (samples) {
      const result = calibratePivot(samples);
      if (result.accepted) this.neckPivotOffset = [...result.offset];
      this.onCalibrationResult?.(result);
    }
    const reference = response.pose_reference;
    if (reference) this.calibration.add(reference);

    const updated = [];
    for (const side of SIDES) {
      const pose = response[`pose_${side}`];
      const trigger = response[`trigger_${side}`];
      // The hands stop while a calibration run is under way: turning the
      // head moves the target by the very arc being measured, and the
      // operator is shaking their head, not reaching.
      if (
        !pose ||
        trigger === undefined ||
        !reference ||
        this.calibration.collecting
      ) {
        continue;
      }
      const adjusted = adjustPose(pose, reference, this);
      const smoothed = this.smoothers[side].smooth(time, [
        ...adjusted.pos,
        ...adjusted.quat,
      ]);
      const arm = teleop.arms[side];
      arm.pos = smoothed.slice(0, 3);
      arm.quat = smoothed.slice(3, 7);
      // Trigger 0 (released) opens the gripper, 1 closes it, like
      // main.py's _map_trigger_to_gripper does in joint angles.
      arm.grip = Math.min(1, Math.max(0, trigger));
      updated.push(side);
    }
    return updated;
  }
}
