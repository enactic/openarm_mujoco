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

// A text panel for the headset: the page's status element is out of sight
// in an immersive session, so the same lines (and the calibration
// instructions and results, which dora-openarm-webxr draws on its own panel)
// are drawn onto a canvas texture on a plane that hangs below the view.
import * as THREE from "three";

const WIDTH = 1024;
const HEIGHT = 320;
const LINE_HEIGHT = 34;

export class XRHud {
  constructor() {
    this.canvas = document.createElement("canvas");
    this.canvas.width = WIDTH;
    this.canvas.height = HEIGHT;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.NoColorSpace;
    const material = new THREE.MeshBasicMaterial({
      map: this.texture,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    });
    // 0.8 m wide, a meter ahead and a bit below eye level: readable
    // without being in the way of the arms.
    this.mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(0.8, (0.8 * HEIGHT) / WIDTH),
      material,
    );
    this.mesh.position.set(0, -0.35, -1);
    this.mesh.renderOrder = 1000;
    this.mesh.visible = false;
    this.text = null;
  }

  setText(text) {
    if (text === this.text) return;
    this.text = text;
    const ctx = this.canvas.getContext("2d");
    ctx.clearRect(0, 0, WIDTH, HEIGHT);
    ctx.fillStyle = "rgba(0, 0, 0, 0.55)";
    ctx.fillRect(0, 0, WIDTH, HEIGHT);
    ctx.fillStyle = "#ffffff";
    ctx.font = "26px monospace";
    ctx.textBaseline = "top";
    text.split("\n").forEach((line, i) => {
      ctx.fillText(line, 20, 16 + i * LINE_HEIGHT);
    });
    this.texture.needsUpdate = true;
  }

  dispose() {
    this.texture.dispose();
    this.mesh.material.dispose();
    this.mesh.geometry.dispose();
  }
}
