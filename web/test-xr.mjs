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
// Headless tests of the WebXR teleoperation pipeline ported from
// dora-openarm-webxr: frame reading (xr-frame.js), the controller pose to
// arm_origin-frame mapping, smoothing and world placement (xr-pose.js), and
// the neck pivot calibration (calibration.js).
// Run with: node --test test-xr.mjs
import { describe, it } from "node:test";
import {
  calibratePivot,
  fitPivotOffset,
  PivotCalibration,
  symmetricEigen3,
} from "./calibration.js";
import {
  eulerZYXToQuat,
  poseWorldToLocal,
  quatError,
  quatMul,
  quatRotVec,
} from "./ik.js";
import { DEFAULT_HOME, TeleopState } from "./teleop.js";
import { readFrame } from "./xr-frame.js";
import {
  adjustPose,
  CONTROLLER_TO_EE,
  DEFAULT_FRAME_OFFSET,
  DEFAULT_NECK_PIVOT_OFFSET,
  HEAD_OFFSET,
  headAnchor,
  OneEuroPoseSmoother,
  ROBOT_ROTATION,
  robotPoseToXR,
  robotToXR,
  worldPlacement,
  XRTeleop,
} from "./xr-pose.js";

const near = (a, b, tol) => Math.abs(a - b) < tol;
const nearVec = (a, b, tol, what = "") =>
  assert.ok(
    a.length === b.length && a.every((v, i) => near(v, b[i], tol)),
    `${what} ${a.map((v) => v.toFixed(4))} != ${b.map((v) => v.toFixed(4))}`,
  );
// Rotation error between two quaternions, in radians (q and -q agree).
const rotError = (a, b) => Math.hypot(...quatError(a, b));

const CONFIG = {
  frameOffset: DEFAULT_FRAME_OFFSET,
  neckPivotOffset: DEFAULT_NECK_PIVOT_OFFSET,
};
const IDENTITY = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
// WebXR pose object from a position and a [w, x, y, z] quaternion.
const xr = (pos, quat = [1, 0, 0, 0]) => ({
  x: pos[0],
  y: pos[1],
  z: pos[2],
  qw: quat[0],
  qx: quat[1],
  qy: quat[2],
  qz: quat[3],
});
const HOME_QUAT = eulerZYXToQuat(0, -Math.PI / 2, 0);

describe("controller pose mapping (main.py: _adjust_pose)", () => {
  it("ROBOT_ROTATION takes WebXR forward/right/up to robot x/-y/z", () => {
    nearVec(quatRotVec(ROBOT_ROTATION, [0, 0, -1]), [1, 0, 0], 1e-12, "fwd");
    nearVec(quatRotVec(ROBOT_ROTATION, [1, 0, 0]), [0, -1, 0], 1e-12, "right");
    nearVec(quatRotVec(ROBOT_ROTATION, [0, 1, 0]), [0, 0, 1], 1e-12, "up");
  });

  it("a hand at the neck pivot lands on the frame offset", () => {
    const pivot = DEFAULT_NECK_PIVOT_OFFSET; // identity headset at the origin
    const { pos } = adjustPose(xr(pivot), IDENTITY, CONFIG);
    nearVec(pos, DEFAULT_FRAME_OFFSET, 1e-12);
  });

  it("a level controller pointing forward gives the home orientation", () => {
    // The keyboard home pose (rpy 0 -90 0 in the arm_origin frame) is what
    // dora-openarm-keyboard and the scene keyframes agree on; the aim pose
    // of a controller held straight ahead must map onto it.
    const { quat } = adjustPose(IDENTITY, IDENTITY, CONFIG);
    assert.ok(rotError(quat, HOME_QUAT) < 1e-9, `quat ${quat}`);
    nearVec(quat, quatMul(ROBOT_ROTATION, CONTROLLER_TO_EE), 1e-12);
  });

  it("robotToXR and robotPoseToXR invert the mapping", () => {
    const reference = xr([0.1, 1.6, -0.2], eulerZYXToQuat(0.1, -0.2, 0.3));
    const target = eulerZYXToQuat(0.2, -1.4, 0.1);
    for (const side of ["left", "right"]) {
      const p = robotToXR(DEFAULT_HOME[side], reference, CONFIG);
      const { pos } = adjustPose(xr(p), reference, CONFIG);
      nearVec(pos, DEFAULT_HOME[side], 1e-12, side);
      const hand = robotPoseToXR(DEFAULT_HOME[side], target, reference, CONFIG);
      const back = adjustPose(hand, reference, CONFIG);
      nearVec(back.pos, DEFAULT_HOME[side], 1e-12, side);
      assert.ok(rotError(back.quat, target) < 1e-9, `${side} orientation`);
    }
  });

  it("the home pose is in front of the chest, hands 30 cm apart", () => {
    const left = robotToXR(DEFAULT_HOME.left, IDENTITY, CONFIG);
    const right = robotToXR(DEFAULT_HOME.right, IDENTITY, CONFIG);
    assert.ok(left[0] < 0 && right[0] > 0, "left hand on the left");
    assert.ok(near(right[0] - left[0], 0.307, 1e-9));
    assert.ok(left[1] < 0, "below the eyes");
    assert.ok(left[2] < -0.2, "in front");
  });

  it("turning the head about the neck pivot leaves the target put", () => {
    // The headset orbits the pivot: for a head rotation q the headset is at
    // pivot - q * offset. With the right offset configured, the target of a
    // stationary hand must not move; with [0, 0, 0] (plain headset
    // subtraction) it drags along the arc.
    const pivotWorld = [0, 1.5, 0];
    const hand = xr([0.2, 1.2, -0.4]);
    // WebXR is y-up: a head turn (yaw) is about y, a nod (pitch) about x.
    const headset = (yaw, pitch, offset) => {
      const q = eulerZYXToQuat(pitch, yaw, 0);
      const swung = quatRotVec(q, offset);
      return xr(
        pivotWorld.map((v, i) => v - swung[i]),
        q,
      );
    };
    const offset = [0.004, -0.081, 0.076];
    const config = { ...CONFIG, neckPivotOffset: offset };
    const still = adjustPose(hand, headset(0, 0, offset), config).pos;
    const turned = adjustPose(hand, headset(0.6, -0.3, offset), config).pos;
    nearVec(turned, still, 1e-12, "with the pivot");

    const plain = { ...CONFIG, neckPivotOffset: [0, 0, 0] };
    const dragged = adjustPose(hand, headset(0.6, 0, offset), plain).pos;
    assert.ok(
      Math.hypot(...dragged.map((v, i) => v - still[i])) > 0.03,
      "plain headset subtraction reads the arc as motion",
    );
  });
});

describe("worldPlacement", () => {
  it("draws a MuJoCo world point where robotToXR puts its target", () => {
    const origin = {
      pos: [0.3, 0.1, 1.15],
      quat: eulerZYXToQuat(0, 0, 0.4),
    };
    const reference = xr([0.05, 1.6, 0.1], eulerZYXToQuat(0, 0.1, -0.2));
    const { pos, quat } = worldPlacement(origin, reference, CONFIG);
    for (const m of [
      [0, 0, 0],
      [0.5, -0.2, 0.9],
      [-1, 1, 2],
    ]) {
      const placed = quatRotVec(quat, m).map((v, i) => v + pos[i]);
      const local = poseWorldToLocal(origin, m, [1, 0, 0, 0]);
      nearVec(placed, robotToXR(local.pos, reference, CONFIG), 1e-9, `${m}`);
    }
  });

  it("draws the anchor's world point at its WebXR point", () => {
    const origin = { pos: [0.185, 0, 1.34], quat: [1, 0, 0, 0] };
    const reference = xr([0.1, 1.5, -0.3], eulerZYXToQuat(0, 0.2, 0));
    const anchor = { world: [0.245, 0, 1.8], xr: [0.1, 1.5, -0.3] };
    const { pos, quat } = worldPlacement(origin, reference, CONFIG, anchor);
    const placed = quatRotVec(quat, anchor.world).map((v, i) => v + pos[i]);
    nearVec(placed, anchor.xr, 1e-12, "camera at the headset");
    // same rotation as without the anchor
    const plain = worldPlacement(origin, reference, CONFIG);
    nearVec(quat, plain.quat, 1e-12, "rotation");
  });

  it("headAnchor puts the head position at the headset", () => {
    const origin = { pos: [0.185, 0, 1.34], quat: eulerZYXToQuat(0, 0, 0.3) };
    const reference = xr([0.1, 1.5, -0.3], eulerZYXToQuat(0, 0.2, 0));
    const anchor = headAnchor(origin, reference);
    const expected = origin.pos.map(
      (v, i) => v + quatRotVec(origin.quat, HEAD_OFFSET)[i],
    );
    nearVec(anchor.world, expected, 1e-12, "head in the world");
    nearVec(anchor.xr, [0.1, 1.5, -0.3], 1e-12, "at the headset");
    const { pos, quat } = worldPlacement(origin, reference, CONFIG, anchor);
    const placed = quatRotVec(quat, anchor.world).map((v, i) => v + pos[i]);
    nearVec(placed, anchor.xr, 1e-12, "placed");
    // the arm base is below the eyes and a little behind them
    const base = quatRotVec(quat, origin.pos).map((v, i) => v + pos[i]);
    assert.ok(base[1] < anchor.xr[1] - 0.2, "below");
  });

  it("turns MuJoCo z-up into WebXR y-up", () => {
    const origin = { pos: [0, 0, 0], quat: [1, 0, 0, 0] };
    const { quat } = worldPlacement(origin, IDENTITY, CONFIG);
    nearVec(quatRotVec(quat, [0, 0, 1]), [0, 1, 0], 1e-12, "up");
    nearVec(quatRotVec(quat, [1, 0, 0]), [0, 0, -1], 1e-12, "forward");
  });
});

describe("OneEuroPoseSmoother (smoothing.py)", () => {
  it("passes the first sample and a non-advancing clock through", () => {
    const s = new OneEuroPoseSmoother();
    const pose = [1, 2, 3, 1, 0, 0, 0];
    assert.deepEqual(s.smooth(0, pose), pose);
    assert.deepEqual(s.smooth(0, [4, 5, 6, 1, 0, 0, 0]), [4, 5, 6, 1, 0, 0, 0]);
  });

  it("lags a step and converges to it, quaternion kept unit", () => {
    const s = new OneEuroPoseSmoother({
      minCutoff: 2,
      beta: 0.04,
      dCutoff: 1.5,
    });
    const q1 = eulerZYXToQuat(0, 0, 1.0);
    s.smooth(0, [0, 0, 0, 1, 0, 0, 0]);
    const first = s.smooth(1 / 72, [1, 0, 0, ...q1]);
    assert.ok(first[0] > 0 && first[0] < 1, `lags: ${first[0]}`);
    assert.ok(near(Math.hypot(...first.slice(3)), 1, 1e-9), "unit quat");
    let last = first;
    for (let i = 2; i <= 72; i++) last = s.smooth(i / 72, [1, 0, 0, ...q1]);
    assert.ok(near(last[0], 1, 1e-3), `converged: ${last[0]}`);
    assert.ok(rotError(last.slice(3), q1) < 1e-2, "rotation converged");
  });

  it("smooths more slowly at low speed than at high speed", () => {
    const step = (speed) => {
      const s = new OneEuroPoseSmoother({ minCutoff: 1, beta: 1, dCutoff: 1 });
      s.smooth(0, [0, 0, 0, 1, 0, 0, 0]);
      s.smooth(0.01, [speed * 0.01, 0, 0, 1, 0, 0, 0]);
      const out = s.smooth(0.02, [speed * 0.02, 0, 0, 1, 0, 0, 0]);
      return out[0] / (speed * 0.02); // fraction of the input reached
    };
    assert.ok(step(10) > step(0.1), "adaptive cutoff");
  });
});

describe("symmetricEigen3", () => {
  it("decomposes a symmetric matrix", () => {
    const A = [4, 1, 2, 1, 3, 0, 2, 0, 5];
    const { values, vectors } = symmetricEigen3(A);
    assert.ok(values[0] <= values[1] && values[1] <= values[2], "ascending");
    for (let k = 0; k < 3; k++) {
      const v = vectors[k];
      assert.ok(near(Math.hypot(...v), 1, 1e-9), "unit");
      const Av = [0, 1, 2].map(
        (i) => A[i * 3] * v[0] + A[i * 3 + 1] * v[1] + A[i * 3 + 2] * v[2],
      );
      nearVec(
        Av,
        v.map((x) => x * values[k]),
        1e-9,
        `eigenpair ${k}`,
      );
    }
    assert.ok(near(values[0] + values[1] + values[2], 12, 1e-9), "trace");
  });
});

// A calibration run: the headset orbits a fixed pivot, at `offset` in its
// own frame, through the given yaw (about WebXR's y) and pitch (about x)
// sweeps in radians, optionally with the body drifting.
function headsetRun(offset, { yaw = 0.45, pitch = 0.45, drift = 0, n = 400 }) {
  const pivotWorld = [0, 1.5, 0];
  const samples = [];
  for (let i = 0; i < n; i++) {
    const t = i / n;
    const phase = 2 * Math.PI * 2 * t; // two sweeps of each over the run
    const y = t < 0.5 ? yaw * Math.sin(2 * phase) : 0;
    const p = t >= 0.5 ? pitch * Math.sin(2 * phase) : 0;
    const q = eulerZYXToQuat(p, y, 0);
    const swung = quatRotVec(q, offset);
    const pos = pivotWorld.map((v, k) => v - swung[k] + drift * t * (k === 0));
    samples.push({ pos, quat: q });
  }
  return samples;
}

describe("neck pivot calibration (calibration.py + main.py policy)", () => {
  const offset = [0.004, -0.081, 0.076];

  it("recovers the offset from a side-to-side and up-and-down run", () => {
    const run = headsetRun(offset, {});
    const { offset: fitted, diagnostics } = fitPivotOffset(
      run.map((s) => s.pos),
      run.map((s) => s.quat),
    );
    nearVec(fitted, offset, 1e-6);
    assert.ok(diagnostics.residualRms < 1e-6, "pivot held still");
    assert.ok(diagnostics.headsetRms > 0.02, "headset moved");
    assert.ok(diagnostics.observability[0] > 0.02, "all axes observed");
    const result = calibratePivot(run);
    assert.equal(result.accepted, true, result.reason);
    nearVec(result.offset, offset, 1e-6);
    assert.equal(result.samples, 400);
  });

  it("rejects a tap", () => {
    const result = calibratePivot(headsetRun(offset, { n: 20 }));
    assert.equal(result.accepted, false);
    assert.match(result.reason, /only 20 headset poses/);
  });

  it("rejects a yaw-only run and asks for the nod that pins the vertical offset", () => {
    const result = calibratePivot(headsetRun(offset, { pitch: 0 }));
    assert.equal(result.accepted, false);
    assert.match(result.reason, /vertical offset; turn it up and down/);
  });

  it("rejects a run where the body moved", () => {
    const result = calibratePivot(headsetRun(offset, { drift: 0.2 }));
    assert.equal(result.accepted, false);
    assert.match(result.reason, /hold the body still/);
  });

  it("rejects a pivot that is not where a neck is", () => {
    const result = calibratePivot(headsetRun([0, 0.1, -0.05], {}));
    assert.equal(result.accepted, false);
    assert.match(result.reason, /not where a neck is/);
  });

  it("PivotCalibration collects for the length of the press, when enabled", () => {
    const off = new PivotCalibration({ enabled: false });
    assert.equal(off.update(true), null);
    assert.equal(off.collecting, false, "disabled: never collects");
    off.add(IDENTITY);
    assert.equal(off.update(false), null);

    const on = new PivotCalibration({ enabled: true, capacity: 3 });
    assert.equal(on.update(false), null, "nothing to fit before a press");
    assert.equal(on.update(true), null);
    assert.equal(on.collecting, true);
    for (let i = 0; i < 5; i++) on.add(xr([i, 0, 0]));
    assert.equal(on.update(true), null, "still held");
    const run = on.update(false);
    assert.equal(run.length, 3, "capacity keeps the newest");
    assert.deepEqual(
      run.map((s) => s.pos[0]),
      [2, 3, 4],
    );
    assert.equal(on.collecting, false);
    assert.equal(on.update(false), null, "a release is one run");
  });
});

// The frame object xr-frame.js builds, from plain values.
function frame({ reference = IDENTITY, right, left, triggers = {} } = {}) {
  const f = { pose_reference: reference };
  if (right) f.pose_right = right;
  if (left) f.pose_left = left;
  for (const [side, v] of Object.entries(triggers)) f[`trigger_${side}`] = v;
  return f;
}

describe("XRTeleop.processFrame (main.py: _process_frame)", () => {
  it("writes the controller poses and triggers into the teleop targets", () => {
    const teleop = new TeleopState();
    const t = new XRTeleop();
    const rightXR = robotToXR([0.3, -0.1, -0.2], IDENTITY, CONFIG);
    const updated = t.processFrame(
      frame({ right: xr(rightXR), triggers: { right: 0.3 } }),
      0,
      teleop,
    );
    assert.deepEqual(updated, ["right"]);
    nearVec(teleop.arms.right.pos, [0.3, -0.1, -0.2], 1e-12);
    assert.ok(rotError(teleop.arms.right.quat, HOME_QUAT) < 1e-9);
    assert.equal(teleop.arms.right.grip, 0.3);
    nearVec(teleop.arms.left.pos, DEFAULT_HOME.left, 1e-12, "left untouched");
  });

  it("needs the headset pose and the trigger to move a hand", () => {
    const teleop = new TeleopState();
    const t = new XRTeleop();
    const f = frame({ right: xr([0, 0, -0.5]), triggers: { right: 0 } });
    f.pose_reference = undefined;
    assert.deepEqual(t.processFrame(f, 0, teleop), []);
    assert.deepEqual(
      t.processFrame(frame({ right: xr([0, 0, -0.5]) }), 0, teleop),
      [],
    );
    nearVec(teleop.arms.right.pos, DEFAULT_HOME.right, 1e-12);
  });

  it("smooths across frames", () => {
    const teleop = new TeleopState();
    const t = new XRTeleop();
    const a = robotToXR([0.2, -0.15, -0.2], IDENTITY, CONFIG);
    const b = robotToXR([0.3, -0.15, -0.2], IDENTITY, CONFIG);
    t.processFrame(frame({ right: xr(a), triggers: { right: 0 } }), 0, teleop);
    t.processFrame(
      frame({ right: xr(b), triggers: { right: 0 } }),
      1 / 72,
      teleop,
    );
    const x = teleop.arms.right.pos[0];
    assert.ok(x > 0.2 && x < 0.3, `between the samples: ${x}`);
  });

  it("stops the hands during a calibration run and applies the result", () => {
    const teleop = new TeleopState();
    const results = [];
    const t = new XRTeleop({
      calibration: true,
      onCalibrationResult: (r) => results.push(r),
    });
    const hand = xr(robotToXR([0.3, -0.1, -0.2], IDENTITY, CONFIG));
    t.processFrame(frame({ right: hand, triggers: { right: 0 } }), 0, teleop);
    const before = [...teleop.arms.right.pos];

    const offset = [0.004, -0.081, 0.076];
    const run = headsetRun(offset, {});
    run.forEach((s, i) => {
      const f = frame({
        reference: xr(s.pos, s.quat),
        right: xr([0.5, 0.5, 0.5]),
        triggers: { right: 0 },
      });
      f.button_y = true;
      assert.deepEqual(t.processFrame(f, (i + 1) / 72, teleop), []);
    });
    nearVec(teleop.arms.right.pos, before, 1e-12, "hands held");
    assert.equal(results.length, 0, "not fitted until the release");

    // release: fitted, accepted and in use from this frame on
    const updated = t.processFrame(
      frame({ right: hand, triggers: { right: 0 } }),
      10,
      teleop,
    );
    assert.equal(results.length, 1);
    assert.equal(results[0].accepted, true, results[0].reason);
    nearVec(t.neckPivotOffset, offset, 1e-6);
    assert.deepEqual(updated, ["right"]);
  });

  it("without calibration the Y button is an ordinary button", () => {
    const teleop = new TeleopState();
    const t = new XRTeleop();
    const f = frame({ right: xr([0, 0, -0.5]), triggers: { right: 0 } });
    f.button_y = true;
    assert.deepEqual(t.processFrame(f, 0, teleop), ["right"]);
    f.button_y = false;
    assert.deepEqual(t.processFrame(f, 1, teleop), ["right"]);
  });
});

// Fakes for the WebXR objects readFrame touches.
function fakeSource(handedness, { profiles, buttons, axes, pose }) {
  return {
    handedness,
    profiles,
    targetRaySpace: { pose },
    gamepad: buttons ? { buttons, axes: axes ?? [] } : null,
  };
}
const fakeFrame = (viewer) => ({
  getViewerPose: () => viewer && { transform: viewer },
  getPose: (space) => space.pose && { transform: space.pose },
});
const transform = (pos, quat = [1, 0, 0, 0]) => ({
  position: { x: pos[0], y: pos[1], z: pos[2] },
  orientation: { x: quat[1], y: quat[2], z: quat[3], w: quat[0] },
});
const button = (value, pressed = value > 0.5) => ({ value, pressed });

describe("readFrame (ar.js: sendFrame)", () => {
  const quest = ["meta-quest-touch-plus", "generic-trigger-squeeze-thumbstick"];

  it("reads the headset, both controllers, triggers, buttons and sticks", () => {
    const session = {
      inputSources: [
        fakeSource("left", {
          profiles: quest,
          pose: transform([-0.2, 1.2, -0.4]),
          buttons: [
            button(0.25),
            button(0),
            button(0),
            button(0),
            button(0),
            button(1),
          ],
          axes: [0, 0, 0.5, -0.25],
        }),
        fakeSource("right", {
          profiles: quest,
          pose: transform([0.2, 1.2, -0.4], HOME_QUAT),
          buttons: [
            button(1),
            button(0.5),
            button(0),
            button(0),
            button(1),
            button(0),
          ],
          axes: [0, 0, 0, 0],
        }),
      ],
    };
    const r = readFrame(session, {}, fakeFrame(transform([0, 1.6, 0])));
    assert.deepEqual(r.pose_reference, {
      ...IDENTITY,
      y: 1.6,
    });
    assert.equal(r.pose_left.x, -0.2);
    assert.ok(near(r.pose_right.qy, HOME_QUAT[2], 1e-12), "orientation xyzw");
    assert.equal(r.trigger_left, 0.25);
    assert.equal(r.trigger_right, 1);
    assert.equal(r.grip_right, 0.5);
    assert.equal(r.button_x, false);
    assert.equal(r.button_y, true);
    assert.equal(r.button_a, true);
    assert.equal(r.button_b, false);
    assert.deepEqual(r.joystick_left, [0, 0, 0.5, -0.25]);
  });

  it("with one controller only the headset is read", () => {
    const session = {
      inputSources: [
        fakeSource("right", {
          profiles: quest,
          pose: transform([0, 0, 0]),
          buttons: [button(1)],
        }),
      ],
    };
    const r = readFrame(session, {}, fakeFrame(transform([0, 1.6, 0])));
    assert.deepEqual(Object.keys(r), ["pose_reference"]);
  });

  it("an unknown profile gives its pose but no trigger, and no headset gives nothing", () => {
    const session = {
      inputSources: [
        fakeSource("left", {
          profiles: ["some-other-controller"],
          pose: transform([0, 0, 0]),
          buttons: [button(1)],
        }),
        fakeSource("none", { profiles: quest, pose: transform([0, 0, 0]) }),
      ],
    };
    const r = readFrame(session, {}, fakeFrame(null));
    assert.deepEqual(Object.keys(r), ["pose_left"]);
  });
});
