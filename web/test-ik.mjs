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

import assert from "node:assert/strict";
// Headless test: drive OpenArm end effectors by pose (position + orientation)
// instead of qpos, using the PoseController from ik.js on the MuJoCo WASM
// bindings, in the v2 cell scene. Run with: node --test test-ik.mjs
import { test } from "node:test";
import loadMuJoCo from "@mujoco/mujoco";
import { PoseController, quatMul } from "./ik.js";
import { loadCellModel } from "./load-model.mjs";

test("pose control converges on offset targets in the cell scene", async (t) => {
  const mujoco = await loadMuJoCo();
  const model = await loadCellModel(mujoco);
  const data = new mujoco.MjData(model);
  const controller = new PoseController(mujoco, model);
  t.after(() => {
    controller.dispose();
    data.delete();
    model.delete();
  });
  t.diagnostic(`model loaded: nq=${model.nq} nv=${model.nv} nu=${model.nu}`);

  // Start from the scene's home keyframe with actuators holding it.
  controller.startFromKeyframe(data);
  controller.syncFrom(data);

  // Pose targets: 8 cm back, 5 cm inward, 8 cm up, rotated 20 deg about z.
  const sides = ["left", "right"];
  const targets = {};
  for (const side of sides) {
    const { pos, quat } = controller.arms[side].pose(data);
    const a = (20 * Math.PI) / 180 / 2;
    targets[side] = {
      pos: [
        pos[0] - 0.08,
        pos[1] + (side === "left" ? -0.05 : 0.05),
        pos[2] + 0.08,
      ],
      quat: quatMul([Math.cos(a), 0, 0, Math.sin(a)], quat),
    };
  }

  // Solve IK once (targets are static) and command the position actuators.
  for (const side of sides) {
    controller.command(data, side, targets[side].pos, targets[side].quat);
    const { errorP, errorR } = controller.arms[side].poseError(
      controller.ikData,
      targets[side].pos,
      targets[side].quat,
    );
    assert.ok(
      Math.hypot(...errorP) < 0.001 && Math.hypot(...errorR) < 0.01,
      `${side} IK solution error too large: ` +
        `${(Math.hypot(...errorP) * 1000).toFixed(3)} mm, ` +
        `${((Math.hypot(...errorR) * 180) / Math.PI).toFixed(3)} deg`,
    );
  }

  // Simulate with gravity/bias compensation on the arm dofs.
  const dt = model.opt.timestep;
  const steps = Math.round(4.0 / dt);
  for (let i = 0; i < steps; i++) {
    controller.applyGravityComp(data);
    mujoco.mj_step(model, data);
  }

  for (const side of sides) {
    const { errorP, errorR } = controller.arms[side].poseError(
      data,
      targets[side].pos,
      targets[side].quat,
    );
    const posErr = Math.hypot(...errorP);
    const rotErr = Math.hypot(...errorR);
    t.diagnostic(
      `${side} after 4 s: pos error = ${(posErr * 1000).toFixed(2)} mm, ` +
        `rot error = ${((rotErr * 180) / Math.PI).toFixed(2)} deg`,
    );
    assert.ok(
      posErr < 0.005,
      `${side} pos error ${(posErr * 1000).toFixed(2)} mm > 5 mm`,
    );
    assert.ok(
      rotErr < 0.05,
      `${side} rot error ${((rotErr * 180) / Math.PI).toFixed(2)} deg > ${((0.05 * 180) / Math.PI).toFixed(1)} deg`,
    );
  }
});
