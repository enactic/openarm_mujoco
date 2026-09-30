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

// Pose integration for keyboard teleoperation, ported from
// dora-openarm-keyboard (src/dora_openarm_keyboard/teleop.py).
//
// Held keys are read as velocities: each step() advances the target pose of
// the key's own arm by speed * dt along every axis whose key is down, so
// both arms can move at once. Holding Shift makes the motion keys drive
// rotation instead of translation. Orientation is integrated in the tool
// frame (r_new = r_cur * delta), which keeps roll, pitch and yaw meaningful
// relative to the gripper rather than the world.
import { eulerZYXToQuat, quatConj, quatError, quatMul } from "./ik.js";
import { GRIP_KEYS, LEFT, MOTION_KEYS, RIGHT, ROTATION_KEY } from "./keymap.js";

// End-effector home pose in the arm_origin frame, identical to
// dora-openarm-keyboard's defaults (the scene's `home` keyframe).
export const DEFAULT_HOME = {
  [LEFT]: [0.216, 0.1535, -0.22],
  [RIGHT]: [0.216, -0.1535, -0.22],
};
const DEFAULT_HOME_RPY_DEG = [0, -90, 0];

const DEFAULT_LINEAR_SPEED = 0.05; // m/s
const DEFAULT_ANGULAR_SPEED = 0.5; // rad/s
const DEFAULT_GRIP_SPEED = 2.0; // fraction/s

const DEFAULT_POS_MIN = [-0.8, -0.8, -0.8];
const DEFAULT_POS_MAX = [0.8, 0.8, 0.8];

function rotvecToQuat(v) {
  const angle = Math.hypot(...v);
  if (angle < 1e-12) return [1, 0, 0, 0];
  const s = Math.sin(angle / 2) / angle;
  return [Math.cos(angle / 2), v[0] * s, v[1] * s, v[2] * s];
}

class ArmState {
  constructor(homePos, homeQuat) {
    this.setHome(homePos, homeQuat);
    this.reset();
  }

  // Redefine the home pose, e.g. from a scene's keyframe, and return to it.
  setHome(homePos, homeQuat) {
    this.homePos = [...homePos];
    this.homeQuat = [...homeQuat];
    this.reset();
  }

  reset() {
    this.pos = [...this.homePos];
    this.quat = [...this.homeQuat];
    this.grip = 0.0; // 0 = fully open, 1 = fully closed
  }

  // Move the target toward home by one bounded step and return whether the
  // arm is now home. The gripper is left where it is: an arm that is holding
  // something carries it home rather than dropping it on the way.
  stepHome(maxDistance, maxAngle) {
    let atHome = true;

    const offset = this.homePos.map((v, i) => v - this.pos[i]);
    const distance = Math.hypot(...offset);
    if (distance <= maxDistance) {
      this.pos = [...this.homePos];
    } else {
      this.pos = this.pos.map(
        (v, i) => v + offset[i] * (maxDistance / distance),
      );
      atHome = false;
    }

    // Integrated in the tool frame, like the manual rotation keys, so the
    // target follows the same geodesic it would under manual control.
    const rotvec = quatError(
      quatMul(quatConj(this.quat), this.homeQuat),
      [1, 0, 0, 0],
    );
    const angle = Math.hypot(...rotvec);
    if (angle <= maxAngle) {
      this.quat = [...this.homeQuat];
    } else {
      this.quat = quatMul(
        this.quat,
        rotvecToQuat(rotvec.map((v) => v * (maxAngle / angle))),
      );
      atHome = false;
    }

    return atHome;
  }
}

export class TeleopState {
  constructor() {
    this.linearSpeed = DEFAULT_LINEAR_SPEED;
    this.angularSpeed = DEFAULT_ANGULAR_SPEED;
    this.gripSpeed = DEFAULT_GRIP_SPEED;
    this.posMin = DEFAULT_POS_MIN;
    this.posMax = DEFAULT_POS_MAX;
    const rad = Math.PI / 180;
    const homeQuat = eulerZYXToQuat(
      ...DEFAULT_HOME_RPY_DEG.map((d) => d * rad),
    );
    this.arms = {
      [LEFT]: new ArmState(DEFAULT_HOME[LEFT], homeQuat),
      [RIGHT]: new ArmState(DEFAULT_HOME[RIGHT], homeQuat),
    };
    this.homing = false;
  }

  reset() {
    for (const side of [LEFT, RIGHT]) this.arms[side].reset();
    this.homing = false;
  }

  startHome() {
    this.homing = true;
  }

  // Abort a home return, leaving both targets where they are.
  cancelHome() {
    this.homing = false;
  }

  // Advance both targets by one timestep of the currently held keys.
  step(dt, heldKeys) {
    if (dt <= 0) return;

    if (this.homing) {
      // A home return owns both targets and moves them at the same speed
      // manual control would, so the arms come back at a speed the operator
      // has already accepted.
      const reached = [LEFT, RIGHT].map((side) =>
        this.arms[side].stepHome(this.linearSpeed * dt, this.angularSpeed * dt),
      );
      this.homing = !reached.every(Boolean);
      return;
    }

    const rotating = heldKeys.has(ROTATION_KEY);
    const linear = { [LEFT]: [0, 0, 0], [RIGHT]: [0, 0, 0] };
    const angular = { [LEFT]: [0, 0, 0], [RIGHT]: [0, 0, 0] };
    const grip = { [LEFT]: 0, [RIGHT]: 0 };

    for (const key of heldKeys) {
      if (Object.hasOwn(MOTION_KEYS, key)) {
        const [side, linearAxis, angularAxis, sign] = MOTION_KEYS[key];
        if (rotating) angular[side][angularAxis] += sign;
        else linear[side][linearAxis] += sign;
      } else if (Object.hasOwn(GRIP_KEYS, key)) {
        const [side, sign] = GRIP_KEYS[key];
        grip[side] += sign;
      }
    }

    for (const side of [LEFT, RIGHT]) {
      const arm = this.arms[side];
      for (let i = 0; i < 3; i++) {
        arm.pos[i] = Math.min(
          this.posMax[i],
          Math.max(
            this.posMin[i],
            arm.pos[i] + linear[side][i] * this.linearSpeed * dt,
          ),
        );
      }
      const rotvec = angular[side].map((v) => v * this.angularSpeed * dt);
      if (rotvec.some((v) => v !== 0)) {
        // tool frame: r_new = r_cur * delta
        arm.quat = quatMul(arm.quat, rotvecToQuat(rotvec));
      }
      if (grip[side]) {
        arm.grip = Math.min(
          1,
          Math.max(0, arm.grip + grip[side] * this.gripSpeed * dt),
        );
      }
    }
  }
}
