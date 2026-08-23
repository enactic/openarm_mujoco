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
// Headless test of keyboard teleoperation in the v2 cell scene: TeleopState
// integration semantics (ported from dora-openarm-keyboard) and the full
// pipeline down to physics — held keys -> arm_origin-frame pose target ->
// world transform -> IK -> actuators -> end effector moves.
// Run with: node --test test-teleop.mjs
import { after, before, describe, it } from "node:test";
import loadMuJoCo from "@mujoco/mujoco";
import { PoseController, poseLocalToWorld, quatError } from "./ik.js";
import { loadCellModel } from "./load-model.mjs";
import { DEFAULT_HOME, TeleopState } from "./teleop.js";

const near = (a, b, tol) => Math.abs(a - b) < tol;

// The its of each describe run in order and share state on purpose: they
// replay one continuous teleop session, each checking the next step.
describe("TeleopState", () => {
  const t = new TeleopState();

  it("hold W for 1 s moves left +x by linear_speed (0.05 m)", () => {
    for (let i = 0; i < 60; i++) t.step(1 / 60, new Set(["w"]));
    assert.ok(near(t.arms.left.pos[0], DEFAULT_HOME.left[0] + 0.05, 1e-9));
  });

  it("right arm is unaffected by W", () => {
    assert.ok(near(t.arms.right.pos[0], DEFAULT_HOME.right[0], 1e-12));
  });

  it("speed scale steps by x1.25 and clamps to [0.1, 10]", () => {
    t.scaleSpeed(1.25);
    assert.ok(near(t.speedScale, 1.25, 1e-12));
    for (let i = 0; i < 100; i++) t.scaleSpeed(1.25);
    assert.ok(near(t.speedScale, 10, 1e-12), "clamped at 10");
    for (let i = 0; i < 100; i++) t.scaleSpeed(1 / 1.25);
    assert.ok(near(t.speedScale, 0.1, 1e-12), "clamped at 0.1");
    t.speedScale = 1;
  });

  it("hold Q for 1 s yaws the left tool 0.5 rad about the tool z axis", () => {
    // Home orientation is pitch -90 deg, so tool z is not world z: the
    // rotation axis expressed in the parent frame must be along x.
    const before = [...t.arms.left.quat];
    for (let i = 0; i < 60; i++) t.step(1 / 60, new Set(["q"]));
    const error = quatError(before, t.arms.left.quat);
    assert.ok(
      near(Math.hypot(...error), 0.5, 1e-6),
      `got ${Math.hypot(...error).toFixed(4)}`,
    );
    const axis = error.map((v) => v / Math.hypot(...error));
    assert.ok(
      near(Math.abs(axis[0]), 1, 1e-3),
      `axis ${axis.map((v) => v.toFixed(3))}`,
    );
  });

  it("grip integrates, clamps at 1, and V reopens", () => {
    for (let i = 0; i < 60; i++) t.step(1 / 60, new Set(["g"]));
    assert.ok(near(t.arms.left.grip, 1, 1e-9), "G closes to 1 (clamped)");
    for (let i = 0; i < 6; i++) t.step(1 / 60, new Set(["v"]));
    assert.ok(near(t.arms.left.grip, 0.8, 1e-9), "V reopens");
  });

  it("reset returns home", () => {
    t.reset();
    assert.ok(near(t.arms.left.pos[0], DEFAULT_HOME.left[0], 1e-12));
    assert.equal(t.arms.left.grip, 0);
  });

  it("position clamps to the workspace bound", () => {
    for (let i = 0; i < 60 * 60; i++) t.step(1 / 60, new Set(["r"]));
    assert.ok(near(t.arms.left.pos[2], 0.8, 1e-9));
    t.reset();
  });
});

describe("full pipeline in the cell scene", () => {
  let mujoco, model, data, controller, teleop, defaultHome;
  let lifterHeight = 0;
  let start, left;

  const simulate = (frames, held) => {
    const dt = model.opt.timestep;
    const stepsPerFrame = Math.round(1 / 60 / dt);
    for (let frame = 0; frame < frames; frame++) {
      teleop.step(1 / 60, held);
      controller.applyTeleop(data, teleop, lifterHeight);
      for (let i = 0; i < stepsPerFrame; i++) {
        controller.applyGravityComp(data);
        mujoco.mj_step(model, data);
      }
    }
  };

  before(async () => {
    mujoco = await loadMuJoCo();
    model = await loadCellModel(mujoco);
    data = new mujoco.MjData(model);
    controller = new PoseController(mujoco, model);
    teleop = new TeleopState();

    // dora-openarm-keyboard's default home target, snapshotted before
    // startFromHome overwrites teleop's home with the measured keyframe pose
    defaultHome = {
      pos: [...teleop.arms.left.pos],
      quat: [...teleop.arms.left.quat],
    };
    controller.startFromHome(data, teleop); // like the app does
  });

  after(() => {
    controller.dispose();
    data.delete();
    model.delete();
  });

  it("cell scene has a lifter", () => {
    assert.ok(controller.lifterActId >= 0);
  });

  it("home keyframe matches the default teleop home pose in the arm_origin frame", () => {
    const origin = controller.originPose(data);
    const homeWorld = poseLocalToWorld(
      origin,
      defaultHome.pos,
      defaultHome.quat,
    );
    const homeError = controller.arms.left.poseError(
      data,
      homeWorld.pos,
      homeWorld.quat,
    );
    assert.ok(
      Math.hypot(...homeError.errorP) < 0.001 &&
        Math.hypot(...homeError.errorR) < 0.01,
      `${(Math.hypot(...homeError.errorP) * 1000).toFixed(2)} mm, ` +
        `${((Math.hypot(...homeError.errorR) * 180) / Math.PI).toFixed(2)} deg`,
    );
  });

  it("2 s of W moves the left EE +10 cm in world x, right EE stays", () => {
    start = controller.arms.left.pose(data);
    simulate(120, new Set(["w"]));
    simulate(60, new Set());
    left = controller.arms.left.pose(data);
    const right = controller.arms.right.pose(data);
    assert.ok(
      near(left.pos[0], start.pos[0] + 0.1, 0.005),
      `dx = ${(left.pos[0] - start.pos[0]).toFixed(4)}`,
    );
    assert.ok(
      near(left.pos[1], start.pos[1], 0.005) &&
        near(left.pos[2], start.pos[2], 0.005),
      "left EE y/z unchanged",
    );
    assert.ok(
      near(right.pos[0], start.pos[0], 0.005),
      "right EE stays at home",
    );
  });

  it("lifter raises the EE while it keeps tracking the arm_origin-frame target", () => {
    // raise the lifter 10 cm: both EEs ride along in world z, targets
    // unchanged in the arm_origin frame
    lifterHeight = 0.1;
    simulate(180, new Set());
    const lifted = controller.arms.left.pose(data);
    assert.ok(
      near(lifted.pos[2], left.pos[2] + 0.1, 0.005),
      `dz = ${(lifted.pos[2] - left.pos[2]).toFixed(4)}`,
    );
    const o2 = controller.originPose(data);
    const targetWorld = poseLocalToWorld(
      o2,
      teleop.arms.left.pos,
      teleop.arms.left.quat,
    );
    const trackError = controller.arms.left.poseError(
      data,
      targetWorld.pos,
      targetWorld.quat,
    );
    assert.ok(
      Math.hypot(...trackError.errorP) < 0.005,
      `${(Math.hypot(...trackError.errorP) * 1000).toFixed(2)} mm`,
    );
  });

  it("applyTeleop skips the IK while the target is unchanged", () => {
    controller.applyTeleop(data, teleop, lifterHeight); // warm the cache
    const solve = controller.solve.bind(controller);
    let calls = 0;
    controller.solve = (...args) => {
      calls++;
      return solve(...args);
    };
    try {
      controller.applyTeleop(data, teleop, lifterHeight);
      assert.equal(calls, 0, "unchanged targets reuse the cached solution");
      teleop.arms.left.pos[0] += 0.01;
      controller.applyTeleop(data, teleop, lifterHeight);
      assert.equal(calls, 1, "only the changed arm solves again");
    } finally {
      delete controller.solve;
    }
  });
});
