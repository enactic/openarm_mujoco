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

// End-to-end test of OpenArm MuJoCo Web with Playwright. The webServer entry
// in playwright.config.js starts serve.mjs automatically.
//
// The tests replay one continuous teleop session on a shared page (serial
// mode): loading the MuJoCo WASM scene is expensive, so they build on each
// other instead of reloading per test.
import { expect, test } from "@playwright/test";

test.describe.configure({ mode: "serial" });

let page;

const leftEE = () =>
  page.evaluate(() => {
    const s = window.__app.mjData.site_xpos;
    const id = window.__app.controller.arms.left.siteId;
    return [s[id * 3], s[id * 3 + 1], s[id * 3 + 2]];
  });
const rightEE = () =>
  page.evaluate(() => {
    const s = window.__app.mjData.site_xpos;
    const id = window.__app.controller.arms.right.siteId;
    return [s[id * 3], s[id * 3 + 1], s[id * 3 + 2]];
  });
// Largest target-tracking error shown in the status panel, in mm. Once it is
// small the arms have settled on their current targets.
const maxTrackError = async () => {
  const text = await page.locator("#status").textContent();
  const errors = [...text.matchAll(/error:\s+([\d.]+) mm/g)].map((m) =>
    Number(m[1]),
  );
  return errors.length === 2 ? Math.max(...errors) : Infinity;
};
const settle = () =>
  expect.poll(maxTrackError, { timeout: 20_000 }).toBeLessThan(1.0);

test.beforeAll(async ({ browser }) => {
  page = await browser.newPage();
  page.on("pageerror", (e) => {
    throw new Error(`page error: ${e}`);
  });
  await page.goto("/");
  await expect(page.locator("#status")).toContainText("error", {
    timeout: 60_000,
  });
  await settle();
});

test.afterAll(() => page?.close());

test("holding W moves the left EE straight +x, right EE stays", async () => {
  const before = await leftEE();
  const beforeRight = await rightEE();
  await page.keyboard.down("w");
  await expect
    .poll(async () => (await leftEE())[0], { timeout: 20_000 })
    .toBeGreaterThan(before[0] + 0.02);
  await page.keyboard.up("w");
  await settle();
  const after = await leftEE();
  expect(Math.abs(after[1] - before[1])).toBeLessThan(0.01);
  expect(Math.abs(after[2] - before[2])).toBeLessThan(0.01);
  const afterRight = await rightEE();
  expect(Math.abs(afterRight[0] - beforeRight[0])).toBeLessThan(0.01);
});

test("holding Y moves the right EE up", async () => {
  const before = await rightEE();
  await page.keyboard.down("y");
  await page.keyboard.down("m"); // close the right gripper along the way
  await expect
    .poll(async () => (await rightEE())[2], { timeout: 20_000 })
    .toBeGreaterThan(before[2] + 0.02);
  await page.keyboard.up("y");
  await page.keyboard.up("m");
  await settle();
});

test("Backspace returns both arms home", async () => {
  const home = await page.evaluate(() => {
    const { pos } = window.__app.targets.left;
    return pos;
  });
  await page.keyboard.press("Backspace");
  await settle();
  // after reset the target is the home pose again; the EE must be back on it
  const ee = await leftEE();
  const target = await page.evaluate(() => window.__app.targets.left.pos);
  expect(Math.hypot(...ee.map((v, i) => v - target[i]))).toBeLessThan(0.01);
  expect(target).not.toEqual(home); // W/Y session had moved the target
});

test("the lifter carries the arms up, Backspace resets it", async () => {
  const before = await leftEE();
  await page.evaluate(() => {
    window.__app.lifterHeight = 0.15; // no UI control yet
  });
  await expect
    .poll(async () => (await leftEE())[2], { timeout: 20_000 })
    .toBeGreaterThan(before[2] + 0.1);
  // Backspace performs the same full reset as the Reset button, lifter
  // included.
  await page.keyboard.press("Backspace");
  await expect
    .poll(() => page.evaluate(() => window.__app.lifterHeight))
    .toBe(0);
  await settle();
});

test("a key released after Shift stops, and Shift turns keys into rotation", async () => {
  const held = () => page.evaluate(() => [...window.__app.held].sort());
  // Shift changes event.key: "w" is released as "W", which it must still
  // let go of. Playwright reports "w" either way, so send that keyup here.
  await page.keyboard.down("w");
  await page.keyboard.down("Shift");
  await page.evaluate(() =>
    window.dispatchEvent(
      new KeyboardEvent("keyup", { key: "W", code: "KeyW", shiftKey: true }),
    ),
  );
  expect(await held()).toEqual(["shift"]);
  await page.keyboard.up("w"); // let Playwright release it too
  const before = await page.evaluate(() => {
    const arm = window.__app.teleop.arms.left;
    return { pos: [...arm.pos], quat: [...arm.quat] };
  });
  await page.keyboard.down("s");
  await page.waitForTimeout(300);
  await page.keyboard.up("s");
  await page.keyboard.up("Shift");
  expect(await held()).toEqual([]);
  const after = await page.evaluate(() => {
    const arm = window.__app.teleop.arms.left;
    return { pos: [...arm.pos], quat: [...arm.quat] };
  });
  expect(after.pos).toEqual(before.pos); // rotated, not moved
  expect(after.quat).not.toEqual(before.quat);
  await page.keyboard.press("Backspace");
  await settle();
});

test("0 walks both arms home, and a motion key cancels it", async () => {
  const home = await page.evaluate(() => [
    ...window.__app.teleop.arms.left.homePos,
  ]);
  await page.keyboard.down("w");
  await expect
    .poll(() => page.evaluate(() => window.__app.teleop.arms.left.pos[0]))
    .toBeGreaterThan(home[0] + 0.02);
  await page.keyboard.up("w");
  await page.keyboard.press("0");
  expect(await page.evaluate(() => window.__app.teleop.homing)).toBe(true);
  await page.keyboard.press("s"); // cancels it well short of home
  const cancelled = await page.evaluate(() => ({
    homing: window.__app.teleop.homing,
    x: window.__app.teleop.arms.left.pos[0],
  }));
  expect(cancelled.homing).toBe(false);
  expect(cancelled.x).toBeGreaterThan(home[0] + 0.005);
  await page.keyboard.press("0");
  await expect
    .poll(() => page.evaluate(() => window.__app.teleop.homing), {
      timeout: 20_000,
    })
    .toBe(false);
  const pos = await page.evaluate(() => window.__app.teleop.arms.left.pos);
  expect(pos).toEqual(home);
  await settle();
});

test("key repeats leave the home return running, a gripper key cancels it", async () => {
  const homing = () => page.evaluate(() => window.__app.teleop.homing);
  await page.keyboard.down("w");
  await expect
    .poll(() => page.evaluate(() => window.__app.teleop.arms.left.pos[0]))
    .toBeGreaterThan(
      await page.evaluate(
        () => window.__app.teleop.arms.left.homePos[0] + 0.02,
      ),
    );
  await page.keyboard.press("0");
  // W is still down: a second down() is the browser's auto-repeat
  await page.keyboard.down("w");
  expect(await homing()).toBe(true);
  await page.keyboard.up("w");
  await page.keyboard.press("x");
  expect(await homing()).toBe(false);
  await page.keyboard.press("Backspace");
  await settle();
});

test("the WebXR button is offered, and says why it cannot start here", async () => {
  // Headless Chromium has no headset: three's VRButton reports that instead
  // of an "ENTER VR" button.
  const button = page.locator("body > button", {
    hasText: /VR NOT SUPPORTED|VR NOT ALLOWED|WEBXR NEEDS HTTPS|ENTER VR/,
  });
  await expect(button).toHaveCount(1);
});

test("controller frames drive the arms through the same pipeline", async () => {
  // No headset in the test browser, so the session is stood up by hand
  // (onSessionStart is what the renderer calls) and frames are fed straight
  // to applyXRFrame, which is where readFrame's output would go.
  await page.keyboard.press("Backspace"); // the previous test left R's target
  await settle();
  const before = await rightEE();
  const beforeLeft = await leftEE();
  const moved = await page.evaluate(async () => {
    const { directPoseToXR } = await import("/web/xr-pose.js");
    const THREE = await import("three");
    const app = window.__app;
    app.onSessionStart();
    const identity = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    // the first frame places the world; the hands go through that placement
    app.applyXRFrame({ pose_reference: identity }, 0);
    const origin = app.controller.originPose(app.mjData);
    // right hand 5 cm ahead of its home target, left hand at home, both in
    // the home orientation
    const hand = (side, dx) => {
      const { homePos, homeQuat } = app.teleop.arms[side];
      const pos = [homePos[0] + dx, homePos[1], homePos[2]];
      return directPoseToXR(pos, homeQuat, app.xrTeleop.placement, origin);
    };
    const frame = {
      pose_reference: identity,
      pose_right: hand("right", 0.05),
      pose_left: hand("left", 0),
      trigger_right: 0.4,
      trigger_left: 0,
    };
    // a second of frames: the One Euro filter has settled by then
    for (let i = 1; i <= 72; i++) app.applyXRFrame(frame, i / 72);
    // where the point viewOffset from the head cameras ended up in the
    // headset's space (the cell's arm_origin frame is the world's)
    const head = app.headPosition();
    const camera = new THREE.Vector3(...head).add(
      new THREE.Vector3(...app.viewOffset),
    );
    app.world.updateMatrixWorld(true);
    app.world.localToWorld(camera);
    // the hands go through the placement from the head cameras themselves
    const { pos, quat } = app.xrTeleop.placement;
    const handsHead = new THREE.Vector3(...head)
      .applyQuaternion(new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]))
      .add(new THREE.Vector3(...pos));
    return {
      target: [...app.teleop.arms.right.pos],
      grip: app.teleop.arms.right.grip,
      placed: app.xrPlaced,
      worldUp: app.world.quaternion.toArray(),
      head,
      camera: camera.toArray(),
      handsHead: handsHead.toArray(),
    };
  });
  expect(moved.placed).toBe(true);
  expect(moved.grip).toBeCloseTo(0.4);
  expect(moved.worldUp).not.toEqual([0, 0, 0, 1]); // turned y-up for the headset
  // between cell.xml's camera_head_left/right, and viewOffset from that
  // drawn at the headset (identity pose here)
  expect(moved.head[0]).toBeCloseTo(0.223, 6);
  expect(moved.head[1]).toBeCloseTo(0, 6);
  expect(moved.head[2]).toBeCloseTo(1.45, 6);
  expect(Math.hypot(...moved.camera)).toBeLessThan(1e-6);
  expect(Math.hypot(...moved.handsHead)).toBeLessThan(1e-6);
  await expect
    .poll(async () => (await rightEE())[0], { timeout: 20_000 })
    .toBeGreaterThan(before[0] + 0.03);
  await settle();
  const after = await rightEE();
  expect(after[0] - before[0]).toBeCloseTo(0.05, 2);
  expect(Math.abs(after[1] - before[1])).toBeLessThan(0.01);
  expect(Math.abs(after[2] - before[2])).toBeLessThan(0.01);
  const afterLeft = await leftEE();
  expect(
    Math.hypot(...afterLeft.map((v, i) => v - beforeLeft[i])),
  ).toBeLessThan(0.01);
  // X resets the environment once per press, B ends the session
  const presses = await page.evaluate(() => {
    const app = window.__app;
    const calls = { reset: 0, end: 0 };
    app.reset = () => {
      calls.reset++;
    };
    app.endSession = () => {
      calls.end++;
    };
    const identity = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    let t = 2;
    for (const button_x of [true, true, false, true, false]) {
      app.applyXRFrame({ pose_reference: identity, button_x }, t++);
    }
    app.applyXRFrame({ pose_reference: identity, button_b: true }, t++);
    delete app.reset;
    delete app.endSession;
    return calls;
  });
  expect(presses).toEqual({ reset: 2, end: 1 });
  // a second of the left thumbstick forward and right raises the view and
  // tilts it down, of the right one forward and right moves it forward and
  // right
  const moved2 = await page.evaluate(async () => {
    const THREE = await import("three");
    const app = window.__app;
    const identity = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    const before = [...app.viewOffset];
    const pitch = app.viewPitch;
    const hands = JSON.stringify(app.xrTeleop.placement);
    // released first: a frame after a pause moves nothing by itself
    app.applyXRFrame({ pose_reference: identity }, 100);
    for (let i = 1; i <= 72; i++) {
      app.applyXRFrame(
        {
          pose_reference: identity,
          joystick_left: [0, 0, 1, -1],
          joystick_right: [0, 0, 1, -1],
        },
        100 + i / 72,
      );
    }
    // the moved eye point (head cameras + viewOffset) is still drawn at the
    // headset, at the origin here
    const eye = new THREE.Vector3(...app.headPosition()).add(
      new THREE.Vector3(...app.viewOffset),
    );
    app.world.updateMatrixWorld(true);
    app.world.localToWorld(eye);
    // and the robot's forward, tilted down by viewPitch, is straight ahead
    const p = app.viewPitch;
    const ahead = new THREE.Vector3(Math.cos(p), 0, -Math.sin(p))
      .applyQuaternion(app.world.quaternion)
      .toArray();
    return {
      by: app.viewOffset.map((v, i) => v - before[i]),
      pitchBy: app.viewPitch - pitch,
      eye: eye.toArray(),
      ahead,
      handsKept: JSON.stringify(app.xrTeleop.placement) === hands,
    };
  });
  // forward, right (-y) and up in the arm_origin frame, 30 degrees down
  const expected = [0.3, -0.3, 0.3];
  for (let i = 0; i < 3; i++) {
    expect(moved2.by[i]).toBeCloseTo(expected[i], 5);
  }
  expect(moved2.pitchBy).toBeCloseTo(Math.PI / 6, 5);
  expect(Math.hypot(...moved2.eye)).toBeLessThan(1e-6);
  expect(moved2.ahead[0]).toBeCloseTo(0, 6);
  expect(moved2.ahead[1]).toBeCloseTo(0, 6);
  expect(moved2.ahead[2]).toBeCloseTo(-1, 6);
  expect(moved2.handsKept).toBe(true); // and the hands reach where they did
  // ending the session puts the world back and keeps the last pose
  const desktopView = await page.evaluate(() => {
    const camera = window.__app.camera;
    return { fov: camera.fov, position: camera.position.toArray() };
  });
  const cameraRestored = await page.evaluate(() => {
    const app = window.__app;
    // what WebXR leaves behind: the headset's pose and field of view
    app.camera.position.set(0, 0, 0);
    app.camera.fov = 100;
    app.camera.updateProjectionMatrix();
    app.onSessionEnd();
    return {
      fov: app.camera.fov,
      position: app.camera.position.toArray(),
      projection: app.camera.projectionMatrix.elements[5],
    };
  });
  expect(cameraRestored.fov).toBe(desktopView.fov);
  for (let i = 0; i < 3; i++) {
    expect(cameraRestored.position[i]).toBeCloseTo(desktopView.position[i], 6);
  }
  expect(cameraRestored.projection).toBeCloseTo(
    1 / Math.tan((desktopView.fov * Math.PI) / 360),
    6,
  );
  expect(
    await page.evaluate(() => window.__app.world.quaternion.toArray()),
  ).toEqual([0, 0, 0, 1]);
  expect(await page.evaluate(() => window.__app.xrTeleop)).toBeNull();
  await page.keyboard.press("Backspace");
  await settle();
});

test("Y opens and closes the HUD", async () => {
  const shown = await page.evaluate(() => {
    const app = window.__app;
    const identity = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    app.onSessionStart();
    let t = 0;
    const press = (button_y) => {
      app.applyXRFrame({ pose_reference: identity, button_y }, t++);
      return app.hudText();
    };
    const out = { closed: press(false) };
    out.open = press(true);
    out.held = press(true); // once per press
    press(false);
    out.closedAgain = press(true);
    app.onSessionEnd();
    return out;
  });
  expect(shown.closed).toBe("press Y for help");
  expect(shown.open).toContain("X: reset");
  expect(shown.open).toContain("error:");
  expect(shown.held).toBe(shown.open);
  expect(shown.closedAgain).toBe("press Y for help");
});

test("the HUD wraps long lines to fit", async () => {
  const drawn = await page.evaluate(() => {
    const app = window.__app;
    app.handMapping = "neck";
    app.calibrationEnabled = true;
    app.onSessionStart();
    // the longest the HUD gets: a run under way and a rejection shown
    app.onCalibrationResult({
      accepted: false,
      reason:
        "the fitted offset [0.012, -0.090, 0.075] is not where a neck is: " +
        "it belongs on the midline, below the eyes and behind them",
    });
    const identity = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    app.applyXRFrame({ pose_reference: identity, button_y: true }, 0);
    app.hud.setText(app.hudText());
    const ctx = app.hud.canvas.getContext("2d");
    const out = {
      text: app.hudText(),
      lines: app.hud.lines,
      widths: app.hud.lines.map((line) => ctx.measureText(line).width),
      canvas: [app.hud.canvas.width, app.hud.canvas.height],
    };
    app.onSessionEnd();
    app.handMapping = "direct";
    app.calibrationEnabled = false;
    return out;
  });
  expect(drawn.text).toContain("CALIBRATING");
  expect(drawn.lines.length).toBeGreaterThan(drawn.text.split("\n").length);
  const [width, height] = drawn.canvas;
  for (const w of drawn.widths) expect(w).toBeLessThanOrEqual(width - 40);
  expect(16 + drawn.lines.length * 34).toBeLessThanOrEqual(height);
});

test("teleop still works after switching scenes", async () => {
  await page.selectOption("#scene-select", "pedestal/bottle_scene.xml");
  await page.waitForFunction(
    () =>
      window.__app.scenePath === "pedestal/bottle_scene.xml" &&
      window.__app.mjModel,
    { timeout: 60_000 },
  );
  await settle();
  const before = await leftEE();
  await page.keyboard.down("r"); // left arm up
  await expect
    .poll(async () => (await leftEE())[2], { timeout: 20_000 })
    .toBeGreaterThan(before[2] + 0.008);
  await page.keyboard.up("r");
});

// Keep these last: the failed load intentionally leaves the app without a
// loaded scene, and the next test checks that state.
test("a missing model file fails with its name, not a parse error", async () => {
  const message = await page.evaluate(() =>
    window.__app.loadScene("does-not-exist.xml").then(
      () => "resolved",
      (e) => String(e.message ?? e),
    ),
  );
  expect(message).toContain("does-not-exist.xml");
  expect(message).toContain("404");
  // a failed load must leave no partial scene behind
  expect(await page.evaluate(() => window.__app.mjModel)).toBeNull();
});

test("Backspace while no scene is loaded is ignored", async () => {
  // Regression: reset() used to call into the null controller and throw
  // (which the pageerror hook above would turn into a test failure).
  await page.keyboard.press("Backspace");
  await page.waitForTimeout(100);
  expect(await page.evaluate(() => window.__app.controller)).toBeNull();
});
