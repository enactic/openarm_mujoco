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
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PoseController } from "./ik.js";
import {
  HELP_TEXT,
  KEYMAP,
  RESET_KEY,
  SPEED_DOWN_KEYS,
  SPEED_UP_KEYS,
} from "./keymap.js";
import { buildVFS } from "./model-vfs.js";
import { TeleopState } from "./teleop.js";

let mujoco;

const SIDES = ["left", "right"];

// Scene and asset files are fetched straight from the repository's v2/
// directory. The path is resolved against the page URL (index.html at
// the repository root), so serve the repository root.
const MODEL_BASE = "v2/";

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

function createCheckerTexture(repeatX = 5, repeatY = 5) {
  const size = 256;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const cells = 8;
  const cell = size / cells;
  for (let y = 0; y < cells; y += 1) {
    for (let x = 0; x < cells; x += 1) {
      const dark = (x + y) % 2 === 0;
      ctx.fillStyle = dark ? "#334455" : "#1a2833";
      ctx.fillRect(x * cell, y * cell, cell, cell);
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(repeatX, repeatY);
  texture.colorSpace = THREE.NoColorSpace;
  texture.anisotropy = 4;
  return texture;
}

function materialHasTexture(model, matId) {
  // mat_texid is nmat × mjNTEXROLE; any non-negative role means textured.
  const texIds = asArray(model.mat_texid);
  if (!texIds || matId < 0) return false;
  const nrole = Math.max(
    1,
    Math.floor(texIds.length / Math.max(model.nmat, 1)),
  );
  const base = matId * nrole;
  for (let r = 0; r < nrole; r += 1) {
    if (texIds[base + r] >= 0) return true;
  }
  return false;
}

function resolveGeomAppearance(model, geomIndex, textureCache) {
  const geomRgba = asArray(model.geom_rgba);
  const geomMatid = asArray(model.geom_matid);
  let rgba = [
    geomRgba[geomIndex * 4],
    geomRgba[geomIndex * 4 + 1],
    geomRgba[geomIndex * 4 + 2],
    geomRgba[geomIndex * 4 + 3],
  ];

  // Defaults match MuJoCo's Phong-like material model (not PBR metalness).
  let shininess = 50;
  let specular = 0.5;
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

    const matShininess = asArray(model.mat_shininess)?.[matId] ?? 0.5;
    const matSpecular = asArray(model.mat_specular)?.[matId] ?? 0.5;
    const matEmission = asArray(model.mat_emission)?.[matId] ?? 0;
    shininess = Math.max(1, matShininess * 100);
    specular = matSpecular;
    emission = matEmission;

    if (materialHasTexture(model, matId)) {
      const texRepeat = asArray(model.mat_texrepeat);
      const repeatX = texRepeat?.[matId * 2] ?? 1;
      const repeatY = texRepeat?.[matId * 2 + 1] ?? 1;
      const key = `${Math.max(repeatX, 1)},${Math.max(repeatY, 1)}`;
      if (!textureCache.has(key)) {
        textureCache.set(
          key,
          createCheckerTexture(Math.max(repeatX, 1), Math.max(repeatY, 1)),
        );
      }
      map = textureCache.get(key);
    }
  }

  return { rgba, shininess, specular, emission, map };
}

class App {
  constructor() {
    this.mjvPerturb = new mujoco.MjvPerturb();
    this.mjvOption = new mujoco.MjvOption();
    this.mjvCamera = new mujoco.MjvCamera();
    this.maxGeoms = 2 ** 14;
    this.meshes = [];
    this.bufferGeometryCache = new Map();
    this.checkerTextureCache = new Map();
    this.teleop = new TeleopState();
    this.held = new Set();
    this.lifterHeight = 0;
    this.statusElement = document.getElementById("status");

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x263238);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    // MuJoCo material rgba is authored for direct display (not Three's sRGB workflow).
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    document.body.appendChild(this.renderer.domElement);

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
    this.initLights();

    window.addEventListener("resize", () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(window.innerWidth, window.innerHeight);
    });
  }

  disposeScene() {
    for (const mesh of this.meshes) {
      this.scene.remove(mesh);
      mesh.material.dispose();
    }
    this.meshes = [];
    for (const geom of this.bufferGeometryCache.values()) geom.dispose();
    this.bufferGeometryCache.clear();
    for (const texture of this.checkerTextureCache.values()) texture.dispose();
    this.checkerTextureCache.clear();
    if (this.markers) {
      for (const side of SIDES) this.scene.remove(this.markers[side]);
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

  initLights() {
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xffffff, 1.2);
    dir.position.set(2, 1, 2);
    this.scene.add(dir);
    const dir2 = new THREE.DirectionalLight(0xffffff, 0.5);
    dir2.position.set(-2, -1, 1);
    this.scene.add(dir2);
  }

  initTargetMarkers() {
    this.markers = {};
    for (const side of SIDES) {
      const marker = new THREE.AxesHelper(0.08);
      this.scene.add(marker);
      this.markers[side] = marker;
    }
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
        2 * (mjvGeom.size[0] || 10),
        2 * (mjvGeom.size[1] || 10),
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
    let shininess = 50;
    let specular = 0.5;
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
        this.checkerTextureCache,
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
    mat.emissive.setRGB(emission, emission, emission);
    if (mat.map !== map) {
      mat.map = map;
      mat.needsUpdate = true;
    }
  }

  update(dt) {
    this.controls.update();
    if (!this.mjModel) return; // scene is loading

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
        this.scene.add(mesh);
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
    if (text === this.lastStatus) return;
    this.lastStatus = text;
    this.statusElement.textContent = text;
  }

  run() {
    let last = null;
    const animate = (now) => {
      // requestAnimationFrame pauses in background tabs: clamp dt so that
      // returning to the tab does not fast-forward all the missed time in
      // one frame.
      const dt = last === null ? 0 : Math.min((now - last) / 1000, 0.1);
      last = now;
      try {
        this.update(dt);
        this.renderer.render(this.scene, this.camera);
      } catch (e) {
        // Stop the loop and rethrow: a swallowed error would repeat at 60 fps
        // with a frozen scene and would never reach pageerror listeners.
        this.statusElement.textContent = `error: ${e.message ?? e}`;
        throw e;
      }
      requestAnimationFrame(animate);
    };
    requestAnimationFrame(animate);
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
  app.run();
}
main();
