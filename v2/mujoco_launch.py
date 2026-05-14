# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import sys

import mujoco
import mujoco.viewer

import time
import openarm_mujoco_v2 as openarm_mujoco
_DEFAULT_SCENE = openarm_mujoco.openarm_cell_xml()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Open a MuJoCo XML in the viewer.")
    parser.add_argument(
        "xml",
        nargs="?",
        default=_DEFAULT_SCENE,
        help=f"Path to MJCF (.xml) file (default: {_DEFAULT_SCENE})",
    )
    parser.add_argument(
        "--keyframe", "-k", default="home",
        help="Name of keyframe to load as initial state",
    )
    parser.add_argument(
        "--static", action="store_true",
        help="Freeze simulation (mj_forward only, no physics stepping)",
    )
    parser.add_argument(
        "--walls",
        action="store_true",
        help="Enable collision walls (default: OFF)",
    )
    parser.add_argument(
        "--no-sheet",
        action="store_true",
        help="Hide sheet mesh",
    )
    args = parser.parse_args()

    xml_path = args.xml

    if not xml_path.lower().endswith(".xml"):
        print(
            f"Error: expected an .xml file, got: {xml_path}", file=sys.stderr)
        return 2

    if not os.path.exists(xml_path):
        print(f"Error: file not found: {xml_path}", file=sys.stderr)
        return 2

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    if args.keyframe is not None:
        key_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
        if key_id == -1:
            print(
                f"Error: keyframe '{args.keyframe}' not found in model.", file=sys.stderr)
            return 2
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        # Sync ctrl to qpos so position actuators hold the keyframe pose.
        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            if jid >= 0:
                data.ctrl[i] = data.qpos[model.jnt_qposadr[jid]]

        mujoco.mj_forward(model, data)

    if not args.walls:
        wall_names = [
            "cell_left_wall_col",
            "cell_right_wall_col",
            "cell_front_wall_col",
            "cell_roof_col",
            "cell_rail_col"
        ]

        for name in wall_names:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom_id != -1:
                # Disable collision
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0

    if args.no_sheet:
        sheet_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "sheet")
        if sheet_id != -1:
            model.geom_rgba[sheet_id][3] = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            if args.static:
                # # In static mode mj_forward never integrates, so ctrl changes
                # # from the viewer sliders would have no effect. Copy each
                # # position actuator's ctrl value directly into qpos so the
                # # viewer stays responsive.
                # for i in range(model.nu):
                #     jid = model.actuator_trnid[i, 0]
                #     if jid >= 0:
                #         data.qpos[model.jnt_qposadr[jid]] = data.ctrl[i]
                mujoco.mj_forward(model, data)
            else:
                mujoco.mj_step(model, data)

                elapsed = time.time() - step_start
                time.sleep(max(0, model.opt.timestep - elapsed))

            viewer.sync()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
