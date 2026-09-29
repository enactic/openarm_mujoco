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

// A head-up display (HUD) for the headset: the page's status element is
// out of sight in an immersive session, so the same lines (and the
// calibration instructions and results, which dora-openarm-webxr draws on
// its own panel) are drawn onto a canvas texture on a plane that hangs
// below the view.
import * as THREE from "three";

const WIDTH = 1024;
const HEIGHT = 640;
const LINE_HEIGHT = 34;
const MARGIN = 20;

// Break `line` at spaces so that each piece fits `maxWidth` as `measure`
// measures it. The pieces after the first are indented two spaces past the
// line's own indentation, so a wrapped line reads as one. A single word
// wider than `maxWidth` is kept whole (and clipped).
export function wrapLine(line, maxWidth, measure) {
  if (measure(line) <= maxWidth) return [line];
  const indent = line.match(/^ */)[0];
  const words = line.slice(indent.length).split(" ");
  const pieces = [];
  let current = indent + words[0];
  for (const word of words.slice(1)) {
    const candidate = `${current} ${word}`;
    if (measure(candidate) <= maxWidth || word === "") {
      current = candidate;
    } else {
      pieces.push(current.trimEnd());
      current = `${indent}  ${word}`;
    }
  }
  pieces.push(current);
  return pieces;
}

export class XRHUD {
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
    // without being in the way of the arms. It hangs from its top edge, so
    // more lines grow it downwards.
    const height = (0.8 * HEIGHT) / WIDTH;
    const geometry = new THREE.PlaneGeometry(0.8, height);
    geometry.translate(0, -height / 2, 0);
    this.mesh = new THREE.Mesh(geometry, material);
    this.mesh.position.set(0, -0.225, -1);
    this.mesh.renderOrder = 1000;
    this.mesh.visible = false;
    this.text = null;
    this.lines = []; // as drawn, after wrapping
  }

  setText(text) {
    if (text === this.text) return;
    this.text = text;
    const ctx = this.canvas.getContext("2d");
    ctx.clearRect(0, 0, WIDTH, HEIGHT);
    ctx.font = "26px monospace";
    ctx.textBaseline = "top";
    const measure = (line) => ctx.measureText(line).width;
    this.lines = text
      .split("\n")
      .flatMap((line) => wrapLine(line, WIDTH - 2 * MARGIN, measure));
    // the backdrop only behind the text, so a one-line hint stays small
    const width = Math.max(...this.lines.map(measure));
    ctx.fillStyle = "rgba(0, 0, 0, 0.55)";
    ctx.fillRect(
      0,
      0,
      Math.min(WIDTH, width + 2 * MARGIN),
      Math.min(HEIGHT, this.lines.length * LINE_HEIGHT + 24),
    );
    ctx.fillStyle = "#ffffff";
    this.lines.forEach((line, i) => {
      ctx.fillText(line, MARGIN, 16 + i * LINE_HEIGHT);
    });
    this.texture.needsUpdate = true;
  }

  dispose() {
    this.texture.dispose();
    this.mesh.material.dispose();
    this.mesh.geometry.dispose();
  }
}
