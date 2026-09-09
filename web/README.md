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

## WebXR (VR controllers)

The page also runs as a WebXR session: open it in the browser of a
headset such as Meta Quest 3 or PICO 4 and press **ENTER VR** at the
bottom of the page. The MuJoCo world is drawn around the operator,
turned y-up for the headset, and the arms follow the controllers: the
trigger closes the gripper, turning the head moves neither the world
nor the targets, the **X** button resets the environment (like the
Reset button and Backspace) and the **B** button leaves the session. A text panel
below the view shows the status lines.

The headset starts where the robot's head would be: OpenArm has none,
so `HEAD_OFFSET` in `xr-pose.js` puts the eyes a human 22 cm above (and
3 cm ahead of) the `arm_origin` site between the shoulder joints, in
every scene. The arms hang below the operator like their own.

The processing that
[dora-openarm-webxr](https://github.com/enactic/dora-openarm-webxr) does
in its Python node runs in the browser here, as direct ports of that
project's sources:

| Module           | Ported from                    | What it does |
|------------------|--------------------------------|--------------|
| `xr-frame.js`    | `static/ar.js`                 | reads the headset pose, the controllers' target-ray poses, triggers, squeezes, thumbsticks and A/B/X/Y buttons out of an `XRFrame` into the frame object the dora client sends. |
| `xr-pose.js`     | `main.py`, `smoothing.py`      | converts a controller pose into an `arm_origin`-frame target (WebXR to robot axes, neck pivot subtraction, aim pose to gripper turn, frame offset), smooths it with the same One Euro filter, and writes it into `TeleopState` for the IK. |
| `calibration.js` | `calibration.py`, `main.py`    | the neck pivot calibration: the least-squares fit of the point the head turns about, and the checks that accept or reject a run. |

The constants (`ROBOT_ROTATION`, the frame offset `[-0.085, 0, -0.14]`,
the neck pivot estimate `[0, -0.075, 0.08]`, the filter parameters) are
the node's defaults. Nothing goes over the network: there is no dora
node, no WebRTC and no camera panel, since the simulation itself is what
the operator sees.

**Neck pivot calibration.** Tick *neck pivot calibration* under
**WebXR** in the panel before entering VR, then hold the **Y** button (left controller), keep the body
still, turn the head side to side twice and up and down twice, and
release. The hands stop following while Y is held. The result (or the
reason a run was rejected, and what to do differently) appears on the
panel in the headset and in the browser console. An accepted offset is
kept in the browser's `localStorage`, so it survives reloads; the box
only says whether the Y button measures, and only from the next session
on.

**HTTPS.** WebXR only runs on a secure page. The GitHub Pages deployment
is one; a page served from `localhost` is too (that is how the
[Immersive Web
Emulator](https://chromewebstore.google.com/detail/immersive-web-emulator/cgffilbpcibhmcfbgggfhfolhkfbhmik)
can drive it on a desktop Chrome without a headset). A headset on the
LAN needs TLS, so `serve.mjs` serves HTTPS when it is given a
certificate, with the same variables dora-openarm-webxr uses. A
self-signed one is enough (the headset browser shows a warning to step
through under "Advanced"):

```sh
name=$(hostname).local  # a name the headset can resolve
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -subj "/CN=${name}" -addext "subjectAltName=DNS:${name}" \
  -keyout server.key -out server.crt
TLS_CERTIFICATE_FILE=server.crt TLS_KEY_FILE=server.key npm run serve
# then open https://${name}:8080/ in the headset
```

## Tests

```sh
npm test              # node:test-based headless tests: IK convergence,
                      # teleop semantics, every scene loading, and the
                      # WebXR pipeline (pose mapping, smoothing, calibration)
npm run test:browser  # Playwright end-to-end test (starts serve.mjs itself);
                      # first run: npx playwright install chromium
```
