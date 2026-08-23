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
// Held keys are read as velocities: each step() advances the target pose by
// speed * scale * dt along every axis whose key is down. Orientation is
// integrated in the tool frame (r_new = r_cur * delta), which keeps roll,
// pitch and yaw meaningful relative to the gripper rather than the world.
import { eulerZYXToQuat, quatMul } from "./ik.js";
import { ANGULAR, GRIP, KEYMAP, LEFT, LINEAR, RIGHT } from "./keymap.js";

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

const MIN_SPEED_SCALE = 0.1;
const MAX_SPEED_SCALE = 10.0;

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
    this.speedScale = 1.0;
  }

  scaleSpeed(factor) {
    this.speedScale = Math.min(
      MAX_SPEED_SCALE,
      Math.max(MIN_SPEED_SCALE, this.speedScale * factor),
    );
    return this.speedScale;
  }

  reset() {
    for (const side of [LEFT, RIGHT]) this.arms[side].reset();
  }

  // Advance both targets by one timestep of the currently held keys.
  step(dt, heldKeys) {
    if (dt <= 0) return;

    const linear = { [LEFT]: [0, 0, 0], [RIGHT]: [0, 0, 0] };
    const angular = { [LEFT]: [0, 0, 0], [RIGHT]: [0, 0, 0] };
    const grip = { [LEFT]: 0, [RIGHT]: 0 };

    for (const key of heldKeys) {
      const binding = KEYMAP[key];
      if (!binding) continue;
      const [side, kind, axis, sign] = binding;
      if (kind === LINEAR) linear[side][axis] += sign;
      else if (kind === ANGULAR) angular[side][axis] += sign;
      else if (kind === GRIP) grip[side] += sign;
    }

    for (const side of [LEFT, RIGHT]) {
      const arm = this.arms[side];
      for (let i = 0; i < 3; i++) {
        arm.pos[i] = Math.min(
          this.posMax[i],
          Math.max(
            this.posMin[i],
            arm.pos[i] +
              linear[side][i] * this.linearSpeed * this.speedScale * dt,
          ),
        );
      }
      const rotvec = angular[side].map(
        (v) => v * this.angularSpeed * this.speedScale * dt,
      );
      if (rotvec.some((v) => v !== 0)) {
        // tool frame: r_new = r_cur * delta
        arm.quat = quatMul(arm.quat, rotvecToQuat(rotvec));
      }
      if (grip[side]) {
        arm.grip = Math.min(
          1,
          Math.max(
            0,
            arm.grip + grip[side] * this.gripSpeed * this.speedScale * dt,
          ),
        );
      }
    }
  }
}
