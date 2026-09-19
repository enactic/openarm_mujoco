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

// Neck pivot calibration, ported from dora-openarm-webxr
// (src/dora_openarm_webxr/calibration.py, and the run collection and the
// accept-or-reject policy of main.py).
//
// The hand targets are made relative to a point in the operator's neck
// rather than the headset itself, because the headset orbits that point as
// the head turns. The offset from the headset to it is an estimate that
// anatomy varies, so this measures it: hold the body still, hold the Y
// button, turn the head, and the one point that stays put through the turn
// is the pivot.
import { quatToMat } from "./ik.js";

// How many headset poses a run may hold. A button left held down stops
// growing here instead of the page's memory; at the headset's display rate
// this is some twenty seconds, well past any deliberate shake.
const CAPACITY = 2000;

// Fewer poses than this and the run is too thin to fit from: a second or
// so of holding, which catches the operator who lets the button go before
// turning their head at all.
const MIN_SAMPLES = 100;

// A run has to turn the head far enough about every axis for the fit to
// see all three offset components. 0.02 is about a 20 degree sweep, which a
// deliberate shake passes twice over.
const MIN_OBSERVABILITY = 0.02;

// The headset's own axes, in the order the offset components come in, and
// the head motion that pins each of them: a rotation cannot see the offset
// along the axis it turns about, so only yawing hides the vertical offset
// and only nodding hides the lateral one.
const OFFSET_AXIS_NAMES = ["lateral", "vertical", "fore-aft"];
const PINNING_MOTION = ["side to side", "up and down", "side to side"];

// How far the fitted pivot may still wander over the run, in meters. Body
// motion lands here, and so does the model error (a neck yaws and nods
// about joints a few centimeters apart rather than one point).
const MAX_RESIDUAL = 0.02;

// Where a neck can be, relative to the eyes, in meters: on the midline, and
// below and behind them.
const MAX_LATERAL = 0.05;
const MAX_VERTICAL = 0.2;
const MAX_FORE_AFT = 0.2;

// Eigendecomposition of a symmetric 3x3 matrix (row-major, 9 numbers) by
// cyclic Jacobi rotations. Returns the eigenvalues ascending and the
// matching unit eigenvectors as rows.
export function symmetricEigen3(matrix) {
  const a = [...matrix];
  const v = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let sweep = 0; sweep < 50; sweep++) {
    const off = Math.hypot(a[1], a[2], a[5]);
    if (off < 1e-15 * Math.max(1, Math.hypot(a[0], a[4], a[8]))) break;
    for (const [p, q] of [
      [0, 1],
      [0, 2],
      [1, 2],
    ]) {
      const apq = a[p * 3 + q];
      if (apq === 0) continue;
      const app = a[p * 3 + p];
      const aqq = a[q * 3 + q];
      const theta = (aqq - app) / (2 * apq);
      const t =
        Math.sign(theta || 1) /
        (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      for (let k = 0; k < 3; k++) {
        // rotate columns p and q of a (and rows, by symmetry)
        const akp = a[k * 3 + p];
        const akq = a[k * 3 + q];
        a[k * 3 + p] = c * akp - s * akq;
        a[k * 3 + q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = a[p * 3 + k];
        const aqk = a[q * 3 + k];
        a[p * 3 + k] = c * apk - s * aqk;
        a[q * 3 + k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const vkp = v[k * 3 + p];
        const vkq = v[k * 3 + q];
        v[k * 3 + p] = c * vkp - s * vkq;
        v[k * 3 + q] = s * vkp + c * vkq;
      }
    }
  }
  const order = [0, 1, 2].sort((i, j) => a[i * 3 + i] - a[j * 3 + j]);
  return {
    values: order.map((i) => a[i * 3 + i]),
    vectors: order.map((i) => [v[i], v[3 + i], v[6 + i]]),
  };
}

function mean3(points) {
  const m = [0, 0, 0];
  for (const p of points) for (let i = 0; i < 3; i++) m[i] += p[i];
  return m.map((v) => v / points.length);
}

// RMS distance of points from their own mean, in meters.
function rmsSpread(points) {
  const m = mean3(points);
  let sum = 0;
  for (const p of points) {
    sum += (p[0] - m[0]) ** 2 + (p[1] - m[1]) ** 2 + (p[2] - m[2]) ** 2;
  }
  return Math.sqrt(sum / points.length);
}

// Fit the headset-to-neck-pivot offset from a run of headset poses
// (calibration.py: fit_pivot_offset).
//
// The pivot in world coordinates is p + R * offset for a headset at
// position p with rotation R. It is the point that does not move while the
// head turns, so the offset that makes those points agree best across the
// run is the fit: minimise sum |(p_i - mean p) + (R_i - mean R) offset|^2,
// a 3x3 linear least squares. Writing A_i = R_i - mean R and b_i = p_i -
// mean p, the normal equations are (sum A_i^T A_i) offset = -sum A_i^T b_i.
//
// `positions` are [x, y, z] and `quats` are [w, x, y, z], both in the same
// world-fixed frame. Returns { offset, diagnostics } where the diagnostics
// hold what the caller needs to judge the fit, never a verdict:
//   samples: how many poses went in.
//   residualRms: how far the fitted pivot still moves, in meters.
//   headsetRms: the same spread for the headset itself, which is what
//     subtracting the headset alone would have carried into the target.
//   observability: the eigenvalues of the normal matrix, averaged over the
//     run and ascending. Each says how much pivot motion a unit of offset
//     along its own axis would have produced.
//   observabilityAxes: the matching unit eigenvectors, in the headset's
//     own frame.
// A direction the head never turned about comes back as zero rather than
// as a division by a rounding error.
export function fitPivotOffset(positions, quats) {
  const samples = positions.length;
  const matrices = quats.map(quatToMat);
  const meanMatrix = new Array(9).fill(0);
  for (const m of matrices) for (let i = 0; i < 9; i++) meanMatrix[i] += m[i];
  for (let i = 0; i < 9; i++) meanMatrix[i] /= samples;
  const meanPosition = mean3(positions);

  // normal = sum A_i^T A_i / N, right = -sum A_i^T b_i / N; averaged over
  // the run so thresholds do not move with how long the operator shook.
  const normal = new Array(9).fill(0);
  const right = [0, 0, 0];
  for (let n = 0; n < samples; n++) {
    const A = matrices[n].map((v, i) => v - meanMatrix[i]);
    const b = positions[n].map((v, i) => v - meanPosition[i]);
    for (let j = 0; j < 3; j++) {
      for (let k = 0; k < 3; k++) {
        let s = 0;
        for (let i = 0; i < 3; i++) s += A[i * 3 + j] * A[i * 3 + k];
        normal[j * 3 + k] += s;
      }
      let s = 0;
      for (let i = 0; i < 3; i++) s += A[i * 3 + j] * b[i];
      right[j] -= s;
    }
  }
  for (let i = 0; i < 9; i++) normal[i] /= samples;
  for (let i = 0; i < 3; i++) right[i] /= samples;

  // Symmetric and positive semi-definite by construction, and its
  // eigenvalues are exactly the per-direction observability.
  const { values, vectors } = symmetricEigen3(normal);
  const largest = values[2];
  const offset = [0, 0, 0];
  for (let k = 0; k < 3; k++) {
    if (!(values[k] > largest * 1e-12)) continue;
    const axis = vectors[k];
    const projected =
      (axis[0] * right[0] + axis[1] * right[1] + axis[2] * right[2]) /
      values[k];
    for (let i = 0; i < 3; i++) offset[i] += axis[i] * projected;
  }

  const pivots = positions.map((p, n) => {
    const m = matrices[n];
    return [
      p[0] + m[0] * offset[0] + m[1] * offset[1] + m[2] * offset[2],
      p[1] + m[3] * offset[0] + m[4] * offset[1] + m[5] * offset[2],
      p[2] + m[6] * offset[0] + m[7] * offset[1] + m[8] * offset[2],
    ];
  });

  return {
    offset,
    diagnostics: {
      samples,
      residualRms: rmsSpread(pivots),
      headsetRms: rmsSpread(positions),
      observability: values,
      observabilityAxes: vectors,
    },
  };
}

// Why a fitted neck pivot offset is not worth applying, or null
// (main.py: _check_pivot_offset).
export function checkPivotOffset(offset, diagnostics) {
  const { samples, observability, observabilityAxes, residualRms } =
    diagnostics;
  if (samples < MIN_SAMPLES) {
    return (
      `only ${samples} headset poses came in; ` +
      "hold Y down for the whole head turn, not just a tap"
    );
  }
  if (observability[0] < MIN_OBSERVABILITY) {
    const weakest = observabilityAxes[0].map(Math.abs);
    const axis = weakest.indexOf(Math.max(...weakest));
    return (
      `the head did not turn enough to see the ${OFFSET_AXIS_NAMES[axis]} ` +
      `offset; turn it ${PINNING_MOTION[axis]} as well`
    );
  }
  if (residualRms > MAX_RESIDUAL) {
    return (
      `the pivot still moved ${(residualRms * 1000).toFixed(0)} mm over ` +
      "the run; hold the body still and turn only the head"
    );
  }
  const [lateral, vertical, foreAft] = offset;
  if (
    Math.abs(lateral) > MAX_LATERAL ||
    !(-MAX_VERTICAL <= vertical && vertical <= 0) ||
    !(0 <= foreAft && foreAft <= MAX_FORE_AFT)
  ) {
    const formatted = offset.map((v) => v.toFixed(3)).join(", ");
    return (
      `the fitted offset [${formatted}] is not where a neck is: it belongs ` +
      "on the midline, below the eyes and behind them"
    );
  }
  return null;
}

// Fit a run and judge it (main.py: _apply_pivot_calibration, minus the
// side effects). Returns { accepted: true, offset, samples, residualMm,
// headsetMm } or { accepted: false, reason }.
export function calibratePivot(samples) {
  const { offset, diagnostics } = fitPivotOffset(
    samples.map((s) => s.pos),
    samples.map((s) => s.quat),
  );
  const reason = checkPivotOffset(offset, diagnostics);
  if (reason !== null) return { accepted: false, reason };
  return {
    accepted: true,
    offset,
    samples: diagnostics.samples,
    residualMm: diagnostics.residualRms * 1000,
    headsetMm: diagnostics.headsetRms * 1000,
  };
}

// Collects headset poses while the operator holds the Y button down
// (main.py: _PivotCalibration).
//
// The button state arrives once per frame rather than as press and release
// events, so the edges are found here. Reading it that way is also the
// failsafe: a controller that falls asleep mid-run stops reporting the
// button at all, the caller reads that as not pressed, and the run ends
// instead of leaving the hands stopped for good.
//
// A disabled one keeps no poses and never stops the hands: measuring is a
// thing the operator sets out to do, not something a press can start by
// accident.
export class PivotCalibration {
  constructor({ enabled = false, capacity = CAPACITY } = {}) {
    this.enabled = enabled;
    this.capacity = capacity;
    this.running = false;
    this.samples = [];
  }

  // Whether a run is under way, so poses belong to it.
  get collecting() {
    return this.running;
  }

  // Take the button state for a frame. The press is the run: it starts the
  // frame the button goes down and ends the frame it comes up, when the
  // run's samples are returned for fitting; null otherwise.
  update(pressed) {
    if (!this.enabled) return null;
    if (pressed) {
      if (!this.running) {
        this.running = true;
        this.samples = [];
      }
      return null;
    }
    const running = this.running;
    this.running = false;
    if (!running || this.samples.length === 0) return null;
    return this.samples;
  }

  // Keep a headset pose (WebXR {x, y, z, qx, qy, qz, qw}) if a run is under
  // way, otherwise drop it.
  add(reference) {
    if (!this.collecting) return;
    if (this.samples.length >= this.capacity) this.samples.shift();
    this.samples.push({
      pos: [reference.x, reference.y, reference.z],
      quat: [reference.qw, reference.qx, reference.qy, reference.qz],
    });
  }
}
