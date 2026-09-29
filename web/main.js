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

import loadMuJoCo from "@mujoco/mujoco";
// OpenArm MuJoCo Web: try the OpenArm in the browser, no installation needed,
// simulated with the MuJoCo WASM bindings.
//
// The end effectors are driven by pose (position + orientation) targets, not
// by qpos: PoseController (ik.js) solves damped-least-squares IK on a
// kinematics-only MjData and feeds the resulting joint targets to the model's
// position actuators.
//
// The pose targets come from TeleopState (teleop.js), which integrates held
// keys exactly like dora-openarm-keyboard: hold to move, tool-frame rotation,
// +/- speed scaling, Backspace to return home.
//
// In a WebXR session the targets come from the VR controllers instead:
// xr-frame.js reads each frame the way dora-openarm-webxr's client does, and
// XRTeleop (xr-pose.js, a port of that project's Python node) converts the
// controller poses into arm_origin-frame targets and writes them into the
// same TeleopState. The MuJoCo world is placed in the headset's reference
// space so that the virtual grippers coincide with the controllers.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { VRButton } from "three/examples/jsm/webxr/VRButton.js";
import { PoseController } from "./ik.js";
import {
  HELP_TEXT,
  KEYMAP,
  RESET_KEY,
  SPEED_DOWN_KEYS,
  SPEED_UP_KEYS,
} from "./keymap.js";
import {
  addMujocoLights,
  materialTexture,
  mujocoLights,
  phongSpecular,
  SHININESS_SCALE,
  skybox,
  texture2D,
} from "./mj-render.js";
import { buildVFS } from "./model-vfs.js";
import { TeleopState } from "./teleop.js";
import { readFrame } from "./xr-frame.js";
import { XRHud } from "./xr-hud.js";
import {
  DEFAULT_NECK_PIVOT_OFFSET,
  DEFAULT_VIEW_OFFSET,
  DEFAULT_VIEW_PITCH,
  HEAD_CAMERAS,
  headAnchor,
  worldPlacement,
  XRTeleop,
} from "./xr-pose.js";

let mujoco;

const SIDES = ["left", "right"];

// Scene and asset files are fetched straight from the repository's v2/
// directory. The path is resolved against the page URL (index.html at
// the repository root), so serve the repository root.
const MODEL_BASE = "v2/";

// The thumbsticks move the VR view at this speed (m/s), within this range
// of the head cameras per axis (arm_origin frame: x forward, y left, z up),
// past this much deflection.
const VIEW_SPEED = 0.3;
const VIEW_OFFSET_RANGE = [
  [-0.5, 0.5],
  [-0.5, 0.5],
  [-0.3, 1.0],
];
const VIEW_STICK_DEADZONE = 0.2;
// ... and tilt it down at this speed (rad/s), within this range.
const VIEW_PITCH_SPEED = (30 * Math.PI) / 180;
const VIEW_PITCH_RANGE = [(-10 * Math.PI) / 180, (60 * Math.PI) / 180];

// A measured neck pivot offset outlives the page (dora-openarm-webxr keeps
// it in neck_pivot.yaml): the whole point of measuring an operator is
// keeping the number they measured.
const NECK_PIVOT_STORAGE_KEY = "openarm-mujoco-web.neck_pivot_offset";

function loadNeckPivotOffset() {
  try {
    const stored = JSON.parse(localStorage.getItem(NECK_PIVOT_STORAGE_KEY));
    if (
      Array.isArray(stored) &&
      stored.length === 3 &&
      stored.every((v) => Number.isFinite(v))
    ) {
      return stored;
    }
  } catch {
    // no storage, or a value that is not ours: use the estimate
  }
  return DEFAULT_NECK_PIVOT_OFFSET;
}

function saveNeckPivotOffset(offset) {
  try {
    localStorage.setItem(NECK_PIVOT_STORAGE_KEY, JSON.stringify(offset));
    return true;
  } catch {
    return false;
  }
}

// fetch() resolves on HTTP errors, so check ok here: a missing model file
// must fail with its name, not as a later, unrelated MuJoCo parse error on
// the 404 body.
async function fetchModelFile(path) {
  const res = await fetch(MODEL_BASE + path);
  if (!res.ok) throw new Error(`${path}: ${res.status} ${res.statusText}`);
  return res;
}

const fetchModelBytes = async (path) =>
  new Uint8Array(await (await fetchModelFile(path)).arrayBuffer());

function asArray(value) {
  if (!value) return value;
  if (typeof value.getView === "function") return value.getView();
  return value;
}

// A plane of size 0 is drawn this large (half-size, in meters) on each side.
const INFINITE_PLANE_HALF_SIZE = 10;

// The Three material parameters for a geom, from its MuJoCo material (or
// MuJoCo's defaults without one). `textures` caches the model's textures by
// id and repeat; `specularRatio` is mujocoLights().specularRatio.
function resolveGeomAppearance(model, geomIndex, textures, specularRatio) {
  const geomRgba = asArray(model.geom_rgba);
  const geomMatid = asArray(model.geom_matid);
  let rgba = [
    geomRgba[geomIndex * 4],
    geomRgba[geomIndex * 4 + 1],
    geomRgba[geomIndex * 4 + 2],
    geomRgba[geomIndex * 4 + 3],
  ];

  // MuJoCo's material defaults.
  let matShininess = 0.5;
  let matSpecular = 0.5;
  let emission = 0;
  let map = null;
  const matId = geomMatid?.[geomIndex] ?? -1;

  if (matId >= 0) {
    // MuJoCo applies material rgba over the geom default (often 0.5 gray).
    const matRgba = asArray(model.mat_rgba);
    rgba = [
      matRgba[matId * 4],
      matRgba[matId * 4 + 1],
      matRgba[matId * 4 + 2],
      matRgba[matId * 4 + 3],
    ];
    matShininess = asArray(model.mat_shininess)?.[matId] ?? matShininess;
    matSpecular = asArray(model.mat_specular)?.[matId] ?? matSpecular;
    emission = asArray(model.mat_emission)?.[matId] ?? emission;

    const texId = materialTexture(model, matId);
    if (texId >= 0) {
      const texRepeat = asArray(model.mat_texrepeat);
      let repeat = [texRepeat[matId * 2], texRepeat[matId * 2 + 1]];
      // Planes are textured uniformly (texuniform, which every plane here
      // sets; the bool array itself cannot be read through the bindings):
      // MuJoCo repeats the texture texrepeat times per half-size.
      if (
        asArray(model.geom_type)[geomIndex] ===
        mujoco.mjtGeom.mjGEOM_PLANE.value
      ) {
        const size = asArray(model.geom_size);
        repeat = repeat.map(
          (r, k) => r * (size[geomIndex * 3 + k] || INFINITE_PLANE_HALF_SIZE),
        );
      }
      const key = `${texId}:${repeat}`;
      if (!textures.has(key)) {
        const base = textures.get(texId) ?? texture2D(model, texId);
        textures.set(texId, base);
        const texture = base.clone(); // shares the pixels
        texture.repeat.set(repeat[0], repeat[1]);
        textures.set(key, texture);
      }
      map = textures.get(key);
    }
  }

  return {
    rgba,
    shininess: Math.max(1, matShininess * SHININESS_SCALE),
    specular: phongSpecular(matSpecular, matShininess, specularRatio),
    emission,
    map,
  };
}

class App {
  constructor() {
    this.mjvPerturb = new mujoco.MjvPerturb();
    this.mjvOption = new mujoco.MjvOption();
    this.mjvCamera = new mujoco.MjvCamera();
    this.maxGeoms = 2 ** 14;
    this.meshes = [];
    this.bufferGeometryCache = new Map();
    this.textureCache = new Map();
    this.sceneLights = []; // the loaded model's, see addMujocoLights
    this.sky = null;
    this.teleop = new TeleopState();
    this.held = new Set();
    this.lifterHeight = 0;
    this.statusElement = document.getElementById("status");

    this.scene = new THREE.Scene();
    // MuJoCo's own background where a scene has no skybox.
    this.scene.background = new THREE.Color(0x000000);
    // Everything MuJoCo draws lives in this group, in MuJoCo's z-up world
    // coordinates. On the desktop it is the identity; in a WebXR session it
    // is moved so that the arm_origin frame lands where the operator is
    // (see placeWorld), since WebXR's reference space is y-up and anchored
    // to the headset.
    this.world = new THREE.Group();
    this.scene.add(this.world);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    // MuJoCo material rgba is authored for direct display (not Three's sRGB workflow).
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    document.body.appendChild(this.renderer.domElement);
    this.renderer.xr.enabled = true;
    // "local" like dora-openarm-webxr: the origin is where the headset was
    // when the session started, which is also what the world is placed from.
    this.renderer.xr.setReferenceSpaceType("local");
    this.renderer.xr.addEventListener("sessionstart", () =>
      this.onSessionStart(),
    );
    this.renderer.xr.addEventListener("sessionend", () => this.onSessionEnd());

    this.camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.01,
      100,
    );
    this.camera.up.set(0, 0, 1); // MuJoCo is z-up
    this.camera.position.set(2.0, -1.3, 1.9); // 3/4 view into the cell

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.4, 0, 1.15);

    // The headset panel rides on the camera, which WebXR moves with the
    // head; on the desktop it is hidden.
    this.scene.add(this.camera);
    this.hud = new XRHud();
    this.camera.add(this.hud.mesh);
    this.handMapping = "direct"; // xr-pose.js's HAND_MAPPINGS
    this.calibrationEnabled = false;
    this.calibrationMessage = null;
    this.xrTeleop = null; // one per session
    this.xrPlaced = false;
    this.xrButtonX = false; // last frame's X, for the press edge
    this.xrButtonY = false; // and Y's
    this.hudExpanded = false; // the full headset panel, toggled by Y
    this.viewOffset = [...DEFAULT_VIEW_OFFSET]; // kept across sessions
    this.viewPitch = DEFAULT_VIEW_PITCH;
    this.xrReference = null; // the headset pose the world is placed from
    this.xrTime = null; // last frame's time, for the thumbstick

    window.addEventListener("resize", () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(window.innerWidth, window.innerHeight);
    });
  }

  disposeScene() {
    for (const mesh of this.meshes) {
      this.world.remove(mesh);
      mesh.material.dispose();
    }
    this.meshes = [];
    for (const geom of this.bufferGeometryCache.values()) geom.dispose();
    this.bufferGeometryCache.clear();
    for (const texture of this.textureCache.values()) texture.dispose();
    this.textureCache.clear();
    for (const light of this.sceneLights) light.removeFromParent();
    this.sceneLights = [];
    if (this.sky) {
      this.world.remove(this.sky);
      this.sky.geometry.dispose();
      this.sky.material.uniforms.sky.value.dispose();
      this.sky.material.dispose();
      this.sky = null;
    }
    if (this.markers) {
      for (const side of SIDES) this.world.remove(this.markers[side]);
      this.markers = null;
    }
    this.mjvScene?.delete();
    this.mjvScene = null;
    this.controller?.dispose();
    this.controller = null;
    this.mjData?.delete();
    this.mjData = null;
    this.mjModel?.delete();
    this.mjModel = null;
  }

  async loadScene(scenePath) {
    this.disposeScene();
    // The scene-switch handler writes "loading…" straight into the status
    // element behind updateStatus's cache; drop the cache, or a scene that
    // settles to the exact same status text would never repaint over it.
    this.lastStatus = null;
    this.scenePath = scenePath;
    this.teleop.reset();
    this.lifterHeight = 0;
    this.held.clear();
    this.xrPlaced = false; // a new scene is placed afresh mid-session

    try {
      const vfs = await buildVFS(mujoco, scenePath, fetchModelBytes, DOMParser);
      try {
        this.mjModel = mujoco.MjModel.from_xml_path(scenePath, vfs);
      } finally {
        vfs.delete();
      }
      this.mjData = new mujoco.MjData(this.mjModel);

      this.controller = new PoseController(mujoco, this.mjModel);
      this.startFromHome();

      // the cell enclosure mesh is drawn see-through so the arms stay visible
      this.transparentGeomIds = new Set();
      const cellVis = mujoco.mj_name2id(
        this.mjModel,
        mujoco.mjtObj.mjOBJ_GEOM.value,
        "cell_vis",
      );
      if (cellVis >= 0) this.transparentGeomIds.add(cellVis);

      this.mjvScene = new mujoco.MjvScene(this.mjModel, this.maxGeoms);
      this.initLighting();
      this.initTargetMarkers();
      this.frameCamera();
    } catch (e) {
      // A partially loaded scene must not survive: update()'s !mjModel guard
      // only protects the render loop when a failure leaves no scene at all.
      this.disposeScene();
      throw e;
    }
  }

  // Frame the camera like MuJoCo's default free camera, which is also what
  // dora-openarm-mujoco sets explicitly for the cell scene: lookat at the
  // model statistics center, azimuth/elevation from the scene's
  // <visual><global>, distance of one model extent (dora uses 3.5 = the cell
  // scene's extent).
  frameCamera() {
    const stat = this.mjModel.stat;
    const g = this.mjModel.vis.global;
    const az = (g.azimuth * Math.PI) / 180;
    const el = (g.elevation * Math.PI) / 180;
    g.delete?.();
    const lookat = [...stat.center];
    const d = Math.max(1.5, stat.extent);
    const forward = [
      Math.cos(el) * Math.cos(az),
      Math.cos(el) * Math.sin(az),
      Math.sin(el),
    ];
    this.camera.position.set(
      lookat[0] - d * forward[0],
      lookat[1] - d * forward[1],
      lookat[2] - d * forward[2],
    );
    this.controls.target.set(...lookat);
  }

  startFromHome() {
    this.simTarget = null; // mjData.time restarts from 0 below
    this.controller.startFromHome(this.mjData, this.teleop);
    this.applyTargets();
  }

  // The scene's own lights, headlight and skybox, as MuJoCo's viewer draws
  // them (mj-render.js). The world-fixed ones go into the world group, so
  // they keep lighting the scene from above when it is turned y-up for a
  // headset.
  initLighting() {
    this.lighting = mujocoLights(this.mjModel);
    this.sceneLights = addMujocoLights(this.lighting, this.world, this.camera);
    this.sky = skybox(this.mjModel, mujoco.mjtTexture.mjTEXTURE_SKYBOX.value);
    if (this.sky) this.world.add(this.sky);
  }

  initTargetMarkers() {
    this.markers = {};
    for (const side of SIDES) {
      const marker = new THREE.AxesHelper(0.08);
      this.world.add(marker);
      this.markers[side] = marker;
    }
  }

  // --- WebXR ---------------------------------------------------------------
  onSessionStart() {
    // Fresh smoothers and calibration per session, like dora-openarm-webxr's
    // session-start; the measured neck pivot offset is the one thing kept.
    this.xrTeleop = new XRTeleop({
      mapping: this.handMapping,
      neckPivotOffset: loadNeckPivotOffset(),
      calibration: this.calibrationEnabled,
      onCalibrationResult: (result) => this.onCalibrationResult(result),
    });
    this.xrPlaced = false;
    this.xrButtonX = false;
    this.xrButtonY = false;
    this.hudExpanded = false;
    this.xrTime = null;
    this.calibrationMessage = null;
    this.hud.mesh.visible = true;
  }

  onSessionEnd() {
    this.xrTeleop = null;
    this.xrPlaced = false;
    this.hud.mesh.visible = false;
    this.world.position.set(0, 0, 0);
    this.world.quaternion.identity();
    // The arms hold the last controller pose; Backspace / Reset return home.
  }

  onCalibrationResult(result) {
    if (result.accepted) {
      const saved = saveNeckPivotOffset(result.offset);
      const formatted = result.offset.map((v) => v.toFixed(3)).join(", ");
      this.calibrationMessage =
        `neck pivot calibration applied from ${result.samples} poses: ` +
        `the pivot held to ${result.residualMm.toFixed(1)} mm while the ` +
        `headset moved ${result.headsetMm.toFixed(1)} mm.\n` +
        `  neck_pivot_offset: [${formatted}]` +
        (saved ? " (saved in this browser)" : "");
    } else {
      this.calibrationMessage = `neck pivot calibration rejected: ${result.reason}`;
    }
    console.log(this.calibrationMessage);
  }

  // Place the MuJoCo world in the headset's reference space, anchored to
  // the headset pose of the first frame: the headset starts viewOffset
  // from the robot's head (between its head cameras), overlooking the arms
  // below, and looking straight ahead looks viewPitch down. A head turn
  // then moves neither the world nor the targets.
  //
  // The hands go through the level placement from the robot's head itself:
  // a hand held where it was reaches the same place whatever the view, and
  // the grippers are drawn moved (and turned about the eyes) by it.
  placeWorld(reference) {
    this.xrReference = reference;
    const origin = this.controller.originPose(this.mjData);
    const head = this.headPosition();
    const place = (viewOffset, pitch) =>
      worldPlacement(
        origin,
        reference,
        this.xrTeleop,
        headAnchor(origin, reference, head, viewOffset),
        pitch,
      );
    const { pos, quat } = place(this.viewOffset, this.viewPitch);
    this.world.position.set(pos[0], pos[1], pos[2]);
    this.world.quaternion.set(quat[1], quat[2], quat[3], quat[0]);
    this.xrTeleop.placement = place(null, 0);
    this.xrPlaced = true;
  }

  // Move the view with the thumbsticks: the left one up and down (pushed
  // forward raises it) and its tilt (right looks further down), the right
  // one forward, back and sideways. The world is placed again from the same
  // headset pose, so only the view moves, and the hands keep their reach.
  adjustView(left, right, dt) {
    if (!this.xrPlaced || dt <= 0) return;
    // xr-standard: the thumbstick is axes[2..3] (touchpad first), -y forward
    const stick = (axes) => {
      if (!axes) return [0, 0];
      const xy = axes.length >= 4 ? [axes[2], axes[3]] : [axes[0], axes[1]];
      return xy.map((v) => (Math.abs(v) > VIEW_STICK_DEADZONE ? v : 0));
    };
    const [leftX, leftY] = stick(left);
    const [rightX, rightY] = stick(right);
    // arm_origin frame: forward is +x, right is -y, up is +z
    const rate = [-rightY, -rightX, -leftY];
    if (rate.every((v) => v === 0) && leftX === 0) return;
    this.viewPitch = Math.min(
      VIEW_PITCH_RANGE[1],
      Math.max(
        VIEW_PITCH_RANGE[0],
        this.viewPitch + leftX * VIEW_PITCH_SPEED * dt,
      ),
    );
    this.viewOffset = this.viewOffset.map((v, i) =>
      Math.min(
        VIEW_OFFSET_RANGE[i][1],
        Math.max(VIEW_OFFSET_RANGE[i][0], v + rate[i] * VIEW_SPEED * dt),
      ),
    );
    this.placeWorld(this.xrReference);
  }

  // World position between the scene's head cameras, or null without them.
  headPosition() {
    const ids = HEAD_CAMERAS.map((name) =>
      mujoco.mj_name2id(this.mjModel, mujoco.mjtObj.mjOBJ_CAMERA.value, name),
    );
    if (ids.some((id) => id < 0)) return null;
    const xpos = this.mjData.cam_xpos;
    return [0, 1, 2].map(
      (k) => ids.reduce((sum, id) => sum + xpos[id * 3 + k], 0) / ids.length,
    );
  }

  // Leave the immersive session (the B button); no-op outside one.
  endSession() {
    this.renderer.xr.getSession()?.end();
  }

  // Feed one frame object (xr-frame.js's readFrame, or a test's) at `time`
  // seconds to the controller pipeline. No-op outside a session.
  applyXRFrame(response, time) {
    if (!this.xrTeleop || !this.controller) return;
    if (!this.xrPlaced && response.pose_reference) {
      this.placeWorld(response.pose_reference);
    }
    // capped, so a stalled frame (or a paused tab) cannot jump the view
    const dt = this.xrTime === null ? 0 : Math.min(0.1, time - this.xrTime);
    this.xrTime = time;
    this.adjustView(response.joystick_left, response.joystick_right, dt);
    this.xrTeleop.processFrame(
      response,
      time,
      this.teleop,
      this.controller.originPose(this.mjData),
    );
    // X resets the environment (the Reset button / Backspace), once per
    // press: the button state comes every frame, so the edge is found here.
    const x = response.button_x === true;
    if (x && !this.xrButtonX) this.reset();
    this.xrButtonX = x;
    // Y shows and hides the full headset panel, once per press, unless the
    // neck pivot calibration has it (hold Y to measure).
    const y = response.button_y === true;
    if (y && !this.xrButtonY && !this.xrTeleop.calibration.enabled) {
      this.hudExpanded = !this.hudExpanded;
    }
    this.xrButtonY = y;
    // Like dora-openarm-webxr's --quit-button: a press ends the session.
    if (response.button_b === true) this.endSession();
  }

  // The headset panel: a one-line hint until Y opens the full panel (always
  // open while the neck pivot calibration holds the Y button).
  hudText() {
    const calibration = this.xrTeleop?.calibration.enabled;
    if (!this.hudExpanded && !calibration) return "press Y for help";
    const lines = [];
    if (this.xrTeleop?.calibration.enabled) {
      const running = this.xrTeleop?.calibration.collecting;
      lines.push(
        running
          ? "CALIBRATING: keep the body still, turn the head side to side " +
              "and up and down, then release Y"
          : "neck pivot calibration: hold Y, turn the head, release",
      );
    }
    if (this.calibrationMessage) lines.push(this.calibrationMessage);
    lines.push(this.lastStatus ?? "");
    const [x, y, z] = this.viewOffset.map((v) => v.toFixed(2));
    const pitch = ((this.viewPitch * 180) / Math.PI).toFixed(0);
    lines.push(
      `view from the head cameras: forward ${x} left ${y} up ${z} m, ` +
        `${pitch} deg down\n` +
        "  left stick: up/down, tilt (sideways)  " +
        "right stick: forward/back/sideways",
    );
    lines.push(`X: reset    B: leave VR${calibration ? "" : "    Y: hide"}`);
    return lines.join("\n");
  }

  applyTargets() {
    // World targets, kept for the markers and the status display.
    this.targets = this.controller.applyTeleop(
      this.mjData,
      this.teleop,
      this.lifterHeight,
    );
    for (const side of SIDES) {
      const m = this.markers?.[side];
      if (m) {
        const { pos, quat } = this.targets[side];
        m.position.set(pos[0], pos[1], pos[2]);
        m.quaternion.set(quat[1], quat[2], quat[3], quat[0]);
      }
    }
  }

  reset() {
    if (!this.controller) return; // scene is loading (or failed to load)
    this.teleop.reset();
    this.lifterHeight = 0;
    this.startFromHome();
  }

  // Build a THREE geometry for a MuJoCo mesh asset (non-indexed, flat faces).
  meshGeometry(meshId) {
    const m = this.mjModel;
    const vertAdr = m.mesh_vertadr[meshId];
    const faceAdr = m.mesh_faceadr[meshId];
    const faceNum = m.mesh_facenum[meshId];
    const normalAdr = m.mesh_normaladr[meshId];
    const positions = new Float32Array(faceNum * 9);
    const normals = new Float32Array(faceNum * 9);
    for (let f = 0; f < faceNum; f++) {
      for (let c = 0; c < 3; c++) {
        const vi = m.mesh_face[(faceAdr + f) * 3 + c];
        const ni = m.mesh_facenormal[(faceAdr + f) * 3 + c];
        for (let k = 0; k < 3; k++) {
          positions[f * 9 + c * 3 + k] = m.mesh_vert[(vertAdr + vi) * 3 + k];
          normals[f * 9 + c * 3 + k] = m.mesh_normal[(normalAdr + ni) * 3 + k];
        }
      }
    }
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geom.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    return geom;
  }

  getBufferGeometry(mjvGeom) {
    const key = JSON.stringify([
      mjvGeom.type,
      [...mjvGeom.size],
      mjvGeom.dataid,
    ]);
    const found = this.bufferGeometryCache.get(key);
    if (found) return found;

    let geom;
    const t = mujoco.mjtGeom;
    if (mjvGeom.type === t.mjGEOM_MESH.value) {
      // for mesh geoms dataid is 2*meshid, +1 when the convex hull is wanted
      // (engine_vis_visualize.c); we always render the full mesh
      geom = this.meshGeometry(mjvGeom.dataid >> 1);
    } else if (mjvGeom.type === t.mjGEOM_PLANE.value) {
      geom = new THREE.PlaneGeometry(
        2 * (mjvGeom.size[0] || INFINITE_PLANE_HALF_SIZE),
        2 * (mjvGeom.size[1] || INFINITE_PLANE_HALF_SIZE),
      );
    } else if (mjvGeom.type === t.mjGEOM_SPHERE.value) {
      geom = new THREE.SphereGeometry(mjvGeom.size[0]);
    } else if (mjvGeom.type === t.mjGEOM_CAPSULE.value) {
      geom = new THREE.CapsuleGeometry(
        mjvGeom.size[0],
        2 * mjvGeom.size[2],
        8,
        16,
      );
      geom.rotateX(0.5 * Math.PI);
    } else if (mjvGeom.type === t.mjGEOM_BOX.value) {
      geom = new THREE.BoxGeometry(
        2 * mjvGeom.size[0],
        2 * mjvGeom.size[1],
        2 * mjvGeom.size[2],
      );
    } else if (mjvGeom.type === t.mjGEOM_CYLINDER.value) {
      geom = new THREE.CylinderGeometry(
        mjvGeom.size[0],
        mjvGeom.size[1],
        2 * mjvGeom.size[2],
        32,
      );
      geom.rotateX(0.5 * Math.PI);
    } else {
      geom = new THREE.BufferGeometry();
    }
    this.bufferGeometryCache.set(key, geom);
    return geom;
  }

  applyGeomAppearance(mesh, g) {
    const mat = mesh.material;
    let rgba = g.rgba;
    // MuJoCo's material defaults, for decor without a geom behind it
    let shininess = 0.5 * SHININESS_SCALE;
    let specular = phongSpecular(0.5, 0.5, this.lighting.specularRatio);
    let emission = 0;
    let map = null;
    if (
      g.objtype === mujoco.mjtObj.mjOBJ_GEOM.value &&
      g.objid >= 0 &&
      this.mjModel
    ) {
      const appearance = resolveGeomAppearance(
        this.mjModel,
        g.objid,
        this.textureCache,
        this.lighting.specularRatio,
      );
      rgba = appearance.rgba;
      shininess = appearance.shininess;
      specular = appearance.specular;
      emission = appearance.emission;
      map = appearance.map;
    }

    mat.color.setRGB(rgba[0], rgba[1], rgba[2]);
    let opacity = rgba[3];
    let transparent = opacity < 0.999;
    let depthWrite = true;
    if (
      g.objtype === mujoco.mjtObj.mjOBJ_GEOM.value &&
      this.transparentGeomIds.has(g.objid)
    ) {
      opacity = 0.15;
      transparent = true;
      depthWrite = false;
    }
    mat.opacity = opacity;
    mat.transparent = transparent;
    mat.depthWrite = depthWrite;
    mat.shininess = shininess;
    mat.specular.setRGB(specular, specular, specular);
    // MuJoCo's emission is a fraction of the color
    mat.emissive.setRGB(
      emission * rgba[0],
      emission * rgba[1],
      emission * rgba[2],
    );
    if (mat.map !== map) {
      mat.map = map;
      mat.needsUpdate = true;
    }
  }

  update(dt, now, xrFrame) {
    // OrbitControls would fight the headset for the camera.
    if (!this.renderer.xr.isPresenting) this.controls.update();
    if (!this.mjModel) return; // scene is loading

    // Controller poses overwrite the targets; held keys still integrate on
    // top (the desktop keyboard keeps working alongside a headset).
    if (xrFrame && this.renderer.xr.isPresenting) {
      const session = this.renderer.xr.getSession();
      const space = this.renderer.xr.getReferenceSpace();
      if (session && space) {
        this.applyXRFrame(readFrame(session, space, xrFrame), now / 1000);
      }
    }
    this.teleop.step(dt, this.held);

    this.applyTargets();

    // Advance physics (with gravity compensation) by the real elapsed time.
    // Physics only moves in model-timestep steps, so carry the target time
    // across frames to keep the fractional remainder.
    this.simTarget = (this.simTarget ?? this.mjData.time) + dt;
    while (this.mjData.time < this.simTarget) {
      this.controller.applyGravityComp(this.mjData);
      mujoco.mj_step(this.mjModel, this.mjData);
    }

    // Sync MuJoCo scene into three.js.
    mujoco.mjv_updateScene(
      this.mjModel,
      this.mjData,
      this.mjvOption,
      this.mjvPerturb,
      this.mjvCamera,
      mujoco.mjtCatBit.mjCAT_ALL.value,
      this.mjvScene,
    );

    const geoms = this.mjvScene.geoms;
    const n = geoms.size();
    for (let i = 0; i < n; i++) {
      const g = geoms.get(i);
      let mesh = this.meshes[i];
      if (!mesh) {
        // DoubleSide: mirrored meshes (scale="1 -1 1") have flipped winding
        const material = new THREE.MeshPhongMaterial({
          side: THREE.DoubleSide,
        });
        mesh = new THREE.Mesh(this.getBufferGeometry(g), material);
        this.meshes.push(mesh);
        this.world.add(mesh);
      }
      mesh.visible = true;
      this.applyGeomAppearance(mesh, g);
      mesh.matrixAutoUpdate = false;
      mesh.matrix.set(
        g.mat[0],
        g.mat[1],
        g.mat[2],
        g.pos[0],
        g.mat[3],
        g.mat[4],
        g.mat[5],
        g.pos[1],
        g.mat[6],
        g.mat[7],
        g.mat[8],
        g.pos[2],
        0,
        0,
        0,
        1,
      );
      mesh.matrixWorldNeedsUpdate = true;
      g.delete();
    }
    for (let i = n; i < this.meshes.length; i++) this.meshes[i].visible = false;
    geoms.delete();

    this.updateStatus();
  }

  updateStatus() {
    const lines = [`speed scale: ${this.teleop.speedScale.toFixed(2)}x`];
    for (const side of SIDES) {
      const { pos, quat } = this.targets[side];
      const { errorP, errorR } = this.controller.arms[side].poseError(
        this.mjData,
        pos,
        quat,
      );
      lines.push(
        `${side.padEnd(5)} error: ${(Math.hypot(...errorP) * 1000).toFixed(1).padStart(6)} mm  ` +
          `${((Math.hypot(...errorR) * 180) / Math.PI).toFixed(1).padStart(5)} deg`,
      );
    }
    // The values are quantized by toFixed, so while settled the text is
    // stable: skip the DOM write (and its style invalidation) entirely.
    const text = lines.join("\n");
    if (text !== this.lastStatus) {
      this.lastStatus = text;
      this.statusElement.textContent = text;
    }
    if (this.hud.mesh.visible) this.hud.setText(this.hudText());
  }

  run() {
    let last = null;
    // The renderer's animation loop, not requestAnimationFrame: inside a
    // WebXR session frames come from the session, and the renderer hands
    // the XRFrame along.
    const animate = (now, xrFrame) => {
      // The loop pauses in background tabs: clamp dt so that returning to
      // the tab does not fast-forward all the missed time in one frame.
      const dt = last === null ? 0 : Math.min((now - last) / 1000, 0.1);
      last = now;
      try {
        this.update(dt, now, xrFrame);
        this.renderer.render(this.scene, this.camera);
      } catch (e) {
        // Stop the loop and rethrow: a swallowed error would repeat at 60 fps
        // with a frozen scene and would never reach pageerror listeners.
        this.renderer.setAnimationLoop(null);
        this.statusElement.textContent = `error: ${e.message ?? e}`;
        throw e;
      }
    };
    this.renderer.setAnimationLoop(animate);
  }
}

// --- Keyboard -------------------------------------------------------------
function setupKeyboard(app) {
  window.addEventListener("keydown", (event) => {
    if (event.ctrlKey || event.altKey || event.metaKey) return;
    const key = event.key.toLowerCase();
    if (key === RESET_KEY) {
      app.reset();
      event.preventDefault();
    } else if (SPEED_UP_KEYS.includes(key)) {
      app.teleop.scaleSpeed(1.25);
    } else if (SPEED_DOWN_KEYS.includes(key)) {
      app.teleop.scaleSpeed(1 / 1.25);
    } else if (KEYMAP[key]) {
      app.held.add(key);
      event.preventDefault();
    }
  });
  // TODO: event.key changes with Shift ('.' releases as '>'), so a KEYMAP
  // punctuation key (';' ',' '.' '/') released while Shift is down — easy to
  // hit, since raising the speed with '+' is Shift+'=' — stays in the held
  // set and keeps driving the arm until the window blurs. Switch keydown and
  // keyup to a shared event.code -> KEYMAP-character translation.
  window.addEventListener("keyup", (event) => {
    app.held.delete(event.key.toLowerCase());
  });
  // Losing focus releases every held key: an unwatched tab never keeps moving.
  window.addEventListener("blur", () => app.held.clear());
}

async function main() {
  mujoco = await loadMuJoCo();
  const app = new App();
  window.__app = app; // for tests and console debugging

  // The scene list lives with the models: v2/scenes.json.
  const { default: defaultScene, scenes } = await (
    await fetchModelFile("scenes.json")
  ).json();
  const select = document.getElementById("scene-select");
  for (const scene of scenes) {
    const option = document.createElement("option");
    option.value = scene;
    option.textContent = scene;
    select.appendChild(option);
  }
  select.value = defaultScene;
  // Disabled until the initial load finishes: loadScene must not run twice
  // concurrently (the loser's WASM objects would leak undeleted).
  select.disabled = true;
  select.onchange = async () => {
    select.disabled = true;
    document.getElementById("status").textContent = "loading…";
    try {
      await app.loadScene(select.value);
    } catch (e) {
      console.error("Scene load failed:", e);
      app.statusElement.textContent = `load failed: ${e.message ?? e}`;
    } finally {
      select.disabled = false;
    }
  };

  try {
    await app.loadScene(defaultScene);
  } catch (e) {
    // Keep going: the rest of the UI must still be wired so that another
    // scene can be picked from the dropdown.
    console.error("Scene load failed:", e);
    app.statusElement.textContent = `load failed: ${e.message ?? e}`;
  }
  select.disabled = false;
  setupKeyboard(app);
  document.getElementById("help").textContent = HELP_TEXT;
  document.getElementById("reset-button").onclick = () => app.reset();

  // WebXR: three's button (bottom center of the page) says "VR NOT
  // SUPPORTED" / "WEBXR NEEDS HTTPS" itself where a session cannot start.
  document.body.appendChild(VRButton.createButton(app.renderer));
  const mapping = document.getElementById("hand-mapping");
  mapping.onchange = () => {
    app.handMapping = mapping.value; // from the next session, like below
  };
  const calibration = document.getElementById("calibration");
  calibration.onchange = () => {
    app.calibrationEnabled = calibration.checked;
    // Takes effect with the next session, like --calibration at startup;
    // the running one keeps the state it started with.
  };
  app.run();
}
main();
