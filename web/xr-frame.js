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

// Read one WebXR frame into the frame object dora-openarm-webxr's client
// sends to its node (static/ar.js: sendFrame), so that xr-pose.js can be a
// port of the node: { pose_reference, pose_left, pose_right, trigger_*,
// grip_*, joystick_*, button_* }, poses as {x, y, z, qx, qy, qz, qw}.

// The controller profiles whose xr-standard gamepad layout is known:
// buttons[0] trigger, [1] squeeze, [4]/[5] A/B on the right hand and X/Y on
// the left.
const KNOWN_PROFILES = ["pico-4u", "meta-quest-touch-plus", "oculus-touch-v3"];

function xrTransform(transform) {
  return {
    x: transform.position.x,
    y: transform.position.y,
    z: transform.position.z,
    qx: transform.orientation.x,
    qy: transform.orientation.y,
    qz: transform.orientation.z,
    qw: transform.orientation.w,
  };
}

export function readFrame(session, space, frame) {
  const response = {};
  // Read before the controller check below: the hand poses are made relative
  // to this pose, so when it is missing the hands are dropped.
  const viewerPose = frame.getViewerPose(space);
  if (viewerPose) response.pose_reference = xrTransform(viewerPose.transform);
  if (session.inputSources.length < 2) return response;
  for (const source of session.inputSources) {
    if (source.handedness === "none") continue;
    const suffix = `_${source.handedness}`;
    // The target ray space is the OpenXR aim pose: its -Z points where the
    // controller points, which xr-pose.js maps onto the gripper axis. Not
    // the grip pose, whose -Z runs along the handle and would turn the
    // handle's tilt into the gripper's.
    const pose = frame.getPose(source.targetRaySpace, space);
    if (pose) response[`pose${suffix}`] = xrTransform(pose.transform);
    const gamepad = source.gamepad;
    if (!gamepad) continue;
    if (source.profiles.some((p) => KNOWN_PROFILES.includes(p))) {
      response[`trigger${suffix}`] = gamepad.buttons[0].value;
      // Sent as its 0..1 value rather than a pressed flag, so that a
      // consumer can pick its own threshold.
      const grip = gamepad.buttons[1];
      if (grip) response[`grip${suffix}`] = grip.value;
      if (source.handedness === "right") {
        response.button_a = gamepad.buttons[4].pressed;
        response.button_b = gamepad.buttons[5].pressed;
      } else {
        response.button_x = gamepad.buttons[4].pressed;
        response.button_y = gamepad.buttons[5].pressed;
      }
    }
    // The whole array: the xr-standard mapping reserves the first axis
    // pair for the touchpad and the second for the thumbstick, and a
    // centred stick reads exactly 0, so no truthiness check here.
    if (gamepad.axes.length >= 2) {
      response[`joystick${suffix}`] = Array.from(gamepad.axes);
    }
  }
  return response;
}
