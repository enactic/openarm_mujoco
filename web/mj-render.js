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

// MuJoCo's own look in Three.js: the model's lights and headlight, its
// textures and its skybox, taken from the compiled model rather than
// guessed, so a scene renders like it does in MuJoCo's viewer.
//
// MuJoCo lights the way fixed-function OpenGL did, in linear color with no
// tone mapping: a surface of color c gets
//   c * (ambient + diffuse * max(n.l, 0)) + specular_mat * specular_light *
//   max(n.h, 0)^(128 shininess)
// summed over the headlight and the model lights. Three's Phong material is
// physically based instead: its diffuse term is c / pi times the light's
// irradiance, and its specular lobe is normalized. The lights here are pi
// times MuJoCo's, and phongSpecular() scales the material so the highlight
// peaks where MuJoCo's does.
import * as THREE from "three";

// MuJoCo's specular exponent is 128 * shininess (mjr's GL_SHININESS).
export const SHININESS_SCALE = 128;

function view(value) {
  return typeof value?.getView === "function" ? value.getView() : value;
}

function rgb(v, i = 0) {
  return new THREE.Color(v[i * 3], v[i * 3 + 1], v[i * 3 + 2]);
}

// The ambient term MuJoCo adds on top of each light's diffuse one: the
// headlight's plus every model light's (their ambient defaults to 0).
// Returns { ambient, diffuse: [{ color, dir, headlight }], specularRatio }:
// dir is the MuJoCo-world direction the light shines in (null for the
// headlight, which shines along the view), and specularRatio how bright the
// highlights are relative to the diffuse light, which phongSpecular needs
// because a Three light has one color for both.
export function mujocoLights(model) {
  const lights = [];
  const ambient = new THREE.Color(0, 0, 0);
  let diffuseSum = 0;
  let specularSum = 0;

  const vis = model.vis;
  const headlight = vis.headlight;
  if (headlight.active) {
    const a = view(headlight.ambient);
    const d = view(headlight.diffuse);
    const s = view(headlight.specular);
    ambient.add(rgb(a));
    lights.push({ color: rgb(d), dir: null, headlight: true });
    diffuseSum += d[0];
    specularSum += s[0];
  }
  headlight.delete?.();
  vis.delete?.();

  // light_active is a bool array, which the WASM bindings cannot hand out
  // (no memory_view<bool>): every model light counts as on.
  const diffuse = view(model.light_diffuse);
  const specular = view(model.light_specular);
  const ambients = view(model.light_ambient);
  const dirs = view(model.light_dir);
  for (let i = 0; i < model.nlight; i++) {
    ambient.add(rgb(ambients, i));
    lights.push({
      color: rgb(diffuse, i),
      dir: [dirs[i * 3], dirs[i * 3 + 1], dirs[i * 3 + 2]],
      headlight: false,
    });
    diffuseSum += diffuse[i * 3];
    specularSum += specular[i * 3];
  }
  return {
    ambient,
    lights,
    specularRatio: diffuseSum > 0 ? specularSum / diffuseSum : 0,
  };
}

// Build the Three lights for `lighting` (mujocoLights()): the ambient and the model
// lights go into `world` (the group holding MuJoCo's z-up world, so they
// turn with it for a headset), the headlight onto `camera`, shining where
// it looks. Returns the objects added, for the next scene to remove.
export function addMujocoLights(lighting, world, camera) {
  const { ambient, lights } = lighting;
  const added = [];
  const add = (parent, object) => {
    parent.add(object);
    added.push(object);
  };
  add(world, new THREE.AmbientLight(ambient, Math.PI));
  for (const light of lights) {
    const three = new THREE.DirectionalLight(light.color, Math.PI);
    if (light.headlight) {
      // along the camera's -z, with the target riding on the camera too
      three.position.set(0, 0, 0);
      three.target.position.set(0, 0, -1);
      add(camera, three);
      add(camera, three.target);
    } else {
      // a directional light only has a direction: from -dir towards 0
      three.position.set(-light.dir[0], -light.dir[1], -light.dir[2]);
      three.target.position.set(0, 0, 0);
      add(world, three);
      add(world, three.target);
    }
  }
  return added;
}

// Three Phong specular color for a MuJoCo material, given
// mujocoLights().specularRatio: Three's Blinn-Phong lobe peaks at
// 0.25 (shininess / 2 + 1) times the specular color per unit of the light's
// (pi-scaled) intensity, MuJoCo's at the material specular times the
// light's specular.
export function phongSpecular(matSpecular, shininess, specularRatio) {
  const exponent = Math.max(1, shininess * SHININESS_SCALE);
  return (matSpecular * specularRatio) / (0.25 * (exponent / 2 + 1));
}

// A texture's pixels from the compiled model as RGBA bytes: MuJoCo keeps
// builtin and file textures alike in tex_data, with 3 or 4 channels.
function texturePixels(model, texId, offset, count) {
  const nchannel = view(model.tex_nchannel)[texId];
  const adr = Number(view(model.tex_adr)[texId]) + offset * nchannel;
  const data = view(model.tex_data);
  const out = new Uint8Array(count * 4);
  for (let i = 0; i < count; i++) {
    for (let k = 0; k < 4; k++) {
      out[i * 4 + k] =
        k < nchannel ? data[adr + i * nchannel + k] : k === 3 ? 255 : 0;
    }
  }
  return out;
}

// A 2D texture as a repeating Three texture, colors as stored (MuJoCo does
// no sRGB decoding).
export function texture2D(model, texId) {
  const width = view(model.tex_width)[texId];
  const height = view(model.tex_height)[texId];
  const texture = new THREE.DataTexture(
    texturePixels(model, texId, 0, width * height),
    width,
    height,
    THREE.RGBAFormat,
  );
  texture.colorSpace = THREE.NoColorSpace;
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.magFilter = THREE.LinearFilter;
  texture.minFilter = THREE.LinearMipmapLinearFilter;
  texture.generateMipmaps = true;
  texture.anisotropy = 4;
  texture.needsUpdate = true;
  return texture;
}

// The texture id a material draws with (its RGB role, else any), or -1.
export function materialTexture(model, matId) {
  const texIds = view(model.mat_texid);
  if (!texIds || matId < 0) return -1;
  const nrole = Math.max(
    1,
    Math.floor(texIds.length / Math.max(model.nmat, 1)),
  );
  const roles = [...texIds.slice(matId * nrole, (matId + 1) * nrole)];
  // mjTEXROLE_RGB comes after mjTEXROLE_USER
  const rgbRole = nrole > 1 ? 1 : 0;
  if (roles[rgbRole] >= 0) return roles[rgbRole];
  return roles.find((id) => id >= 0) ?? -1;
}

// The model's skybox (a texture of type mjTEXTURE_SKYBOX) as a sphere to add
// to the world group, or null without one. MuJoCo keeps a cube texture as
// six square faces in GL cube map order (+x, -x, +y, -y, +z, -z) and draws
// the skybox with the cube's +y as the world's +z; the sphere's shader does
// that turn itself, so the sky stays put when the world group is turned for
// a headset.
export function skybox(model, skyboxType, radius = 50) {
  const types = view(model.tex_type);
  let texId = -1;
  for (let t = 0; t < model.ntex; t++) {
    if (types[t] === skyboxType) {
      texId = t;
      break;
    }
  }
  if (texId < 0) return null;
  const width = view(model.tex_width)[texId];
  const height = view(model.tex_height)[texId];
  const faceSize = width;
  const faces = height === 6 * width ? 6 : 1;
  const images = [];
  for (let f = 0; f < 6; f++) {
    // a single square repeats on every face
    const face = faces === 6 ? f : 0;
    const pixels = texturePixels(
      model,
      texId,
      face * faceSize * faceSize,
      faceSize * faceSize,
    );
    const canvas = document.createElement("canvas");
    canvas.width = faceSize;
    canvas.height = faceSize;
    const ctx = canvas.getContext("2d");
    ctx.putImageData(
      new ImageData(new Uint8ClampedArray(pixels.buffer), faceSize, faceSize),
      0,
      0,
    );
    images.push(canvas);
  }
  const cube = new THREE.CubeTexture(images);
  cube.colorSpace = THREE.NoColorSpace;
  cube.needsUpdate = true;
  const material = new THREE.ShaderMaterial({
    uniforms: { sky: { value: cube } },
    vertexShader: /* glsl */ `
      varying vec3 vDir;
      void main() {
        vDir = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform samplerCube sky;
      varying vec3 vDir;
      void main() {
        // MuJoCo z-up to the cube's y-up
        gl_FragColor = textureCube(sky, vec3(vDir.x, vDir.z, -vDir.y));
      }
    `,
    side: THREE.BackSide,
    depthWrite: false,
  });
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 32, 16),
    material,
  );
  mesh.renderOrder = -1;
  mesh.frustumCulled = false;
  return mesh;
}
