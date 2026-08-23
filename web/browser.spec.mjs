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

test("holding O moves the right EE up", async () => {
  const before = await rightEE();
  await page.keyboard.down("o");
  await page.keyboard.down(";"); // close the right gripper along the way
  await expect
    .poll(async () => (await rightEE())[2], { timeout: 20_000 })
    .toBeGreaterThan(before[2] + 0.02);
  await page.keyboard.up("o");
  await page.keyboard.up(";");
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
  expect(target).not.toEqual(home); // W/O session had moved the target
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
