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
// Headless test: every v2 scene loads through the VFS, starts at its home
// pose without yanking, and pose control still reaches a small offset target.
// Run with: node --test test-scenes.mjs
import fs from "node:fs";
import path from "node:path";
import { test } from "node:test";
import loadMuJoCo from "@mujoco/mujoco";
import { DOMParser } from "@xmldom/xmldom";
import { PoseController, poseLocalToWorld } from "./ik.js";
import { loadSceneModel } from "./load-model.mjs";
import { buildVFS } from "./model-vfs.js";
import { TeleopState } from "./teleop.js";

// The same scene list the app's dropdown shows.
const { scenes: SCENES } = JSON.parse(
  fs.readFileSync(
    path.resolve(import.meta.dirname, "../v2/scenes.json"),
    "utf8",
  ),
);

const mujoco = await loadMuJoCo();

test("buildVFS ignores file references inside XML comments", async () => {
  const files = {
    "scene.xml": `<mujoco>
      <compiler meshdir="assets"/>
      <!-- <geom type="mesh" file="ghost.stl"/> -->
      <asset><mesh file="real.stl"/></asset>
    </mujoco>`,
    "assets/real.stl": new Uint8Array([0]),
  };
  const requested = [];
  const vfs = await buildVFS(
    mujoco,
    "scene.xml",
    async (p) => {
      requested.push(p);
      if (!(p in files)) throw new Error(`${p}: missing`);
      const f = files[p];
      return typeof f === "string" ? new TextEncoder().encode(f) : f;
    },
    DOMParser,
  );
  vfs.delete();
  assert.ok(requested.includes("assets/real.stl"), "real reference is loaded");
  assert.ok(
    !requested.some((p) => p.includes("ghost")),
    "commented-out reference is not loaded",
  );
});

test("buildVFS rejects file references it cannot resolve correctly", async () => {
  // texture/hfield/skin files resolve against texturedir/assetdir, which the
  // walker does not implement: it must fail loudly, not fetch a wrong path.
  const scene = `<mujoco><asset><texture name="t" file="wood.png"/></asset></mujoco>`;
  await assert.rejects(
    buildVFS(
      mujoco,
      "scene.xml",
      async () => new TextEncoder().encode(scene),
      DOMParser,
    ),
    /unsupported file reference <texture file="wood\.png">/,
  );
});

for (const scene of SCENES) {
  test(scene, async (t) => {
    const model = await loadSceneModel(mujoco, scene);
    const data = new mujoco.MjData(model);
    const controller = new PoseController(mujoco, model);
    const teleop = new TeleopState();
    t.after(() => {
      controller.dispose();
      data.delete();
      model.delete();
    });

    // start from home (keyframe if present, else IK of the teleop default),
    // exactly like the app does
    controller.startFromHome(data, teleop);

    // command the home pose + a 5 cm world-z offset on the left arm; simulate
    teleop.arms.left.pos[2] += 0.05;
    const dt = model.opt.timestep;
    const stepsPerFrame = Math.max(1, Math.round(1 / 60 / dt));
    for (let frame = 0; frame < 120; frame++) {
      controller.applyTeleop(data, teleop, 0);
      for (let i = 0; i < stepsPerFrame; i++) {
        controller.applyGravityComp(data);
        mujoco.mj_step(model, data);
      }
    }

    const o = controller.originPose(data);
    const tL = poseLocalToWorld(o, teleop.arms.left.pos, teleop.arms.left.quat);
    const { errorP } = controller.arms.left.poseError(data, tL.pos, tL.quat);
    const tR = poseLocalToWorld(
      o,
      teleop.arms.right.pos,
      teleop.arms.right.quat,
    );
    const { errorP: errorPR } = controller.arms.right.poseError(
      data,
      tR.pos,
      tR.quat,
    );
    const error = Math.hypot(...errorP);
    const errorR = Math.hypot(...errorPR);
    t.diagnostic(
      `nq=${model.nq} nkey=${model.nkey} lifter=${controller.lifterActId >= 0 ? "y" : "n"} ` +
        `left error=${(error * 1000).toFixed(1)}mm right error=${(errorR * 1000).toFixed(1)}mm`,
    );
    assert.ok(
      error < 0.01,
      `left error ${(error * 1000).toFixed(1)} mm > 10 mm`,
    );
    assert.ok(
      errorR < 0.01,
      `right error ${(errorR * 1000).toFixed(1)} mm > 10 mm`,
    );
  });
}
