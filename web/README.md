# OpenArm MuJoCo Web

OpenArm MuJoCo Web runs the OpenArm bimanual robot in the browser: open
the page and try the robot — no installation needed. Physics is
simulated with the official MuJoCo WASM bindings (`@mujoco/mujoco`), and
the arms are driven by end-effector **pose** (position + orientation)
instead of qpos.

How it works:

1. `ik.js` — `PoseController` runs damped-least-squares differential IK on a
   kinematics-only `MjData` scratch state (`mj_kinematics` + `mj_comPos` +
   `mj_jacSite`), yielding joint targets for a requested pose of the
   `left_ee_control_point` / `right_ee_control_point` sites.
2. The joint targets feed the model's position actuators via `data.ctrl`.
3. Gravity/bias compensation (`qfrc_applied = qfrc_bias` on the arm dofs)
   removes steady-state sag from the low-gain actuators.

The scene XMLs and mesh assets are fetched from the repository's `v2/`
directory into an `MjVFS`, following each XML's `<model file>` /
`meshdir` references recursively. A dropdown switches between all v2
scenes (`openarm_bimanual.xml`, `cell/*`, `pedestal/*`); each starts from
its `home` keyframe (or the IK home solution when the scene has none),
and Backspace returns to that scene's own home pose. Cell scenes have a
lifter (no UI control yet — a keyboard binding is planned), and the cell
enclosure is drawn see-through.

## Usage

There is no build step or bundler: the page is plain HTML + ES modules.
The page itself is the repository top-level `index.html`; the modules it
loads live in `web/`. `three` and `@mujoco/mujoco` (including
`mujoco.wasm`) are loaded from the jsDelivr npm CDN via an import map
that `index.html` builds at runtime from `web/package-lock.json` — the
lockfile is the single source for dependency versions, shared with the
tests below. Any static file server works and no npm install is needed
to run the page. Serve the **repository root** (the page fetches models
from `v2/`):

```sh
npm run serve  # node serve.mjs: serves the repo root on port 8080
# then open http://localhost:8080/
```

`npm install` is only needed for the tests below (it pulls the same
pinned `three` / `@mujoco/mujoco` versions for Node, plus Playwright).

Drive the arms with the keyboard (same bindings and semantics as
[dora-openarm-keyboard](https://github.com/enactic/dora-openarm-keyboard):
hold to move, tool-frame rotation, `+`/`-` speed scale, `Backspace` to
return home, and losing tab focus releases every held key). `keymap.js` and
`teleop.js` are direct ports of dora-openarm-keyboard's `keymap.py` and
`teleop.py`, including its home pose (`0.216 ±0.1535 -0.22`, rpy
`0 -90 0` in the `arm_origin` frame); the simulation starts from the IK
solution of that pose.

| | Left arm | Right arm |
|---|---|---|
| +X / -X | W / S | U / J |
| +Y / -Y | A / D | H / K |
| +Z / -Z | R / F | O / L |
| +Pitch / -Pitch | E / C | I / , |
| +Yaw / -Yaw | Q / Z | Y / N |
| +Roll / -Roll | T / B | P / / |
| Gripper close / open | G / V | ; / . |

## Tests

```sh
npm test              # node:test-based headless tests: IK convergence,
                      # teleop semantics, and every scene loading
npm run test:browser  # Playwright end-to-end test (starts serve.mjs itself);
                      # first run: npx playwright install chromium
```
