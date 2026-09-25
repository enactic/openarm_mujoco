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

"""Launcher mode tests without opening windows."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import mujoco
import numpy as np

from openarm_mujoco.v2 import launch


class LaunchTests(unittest.TestCase):
    """Check native viewer dispatch and unchanged static semantics."""

    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string("""
            <mujoco>
              <option timestep="0.001" integrator="implicitfast"/>
              <worldbody><body><joint name="j"/>
                <geom type="sphere" size="0.1" mass="1"/>
              </body></worldbody>
              <actuator><position joint="j" kp="10"/></actuator>
              <keyframe><key name="home" qpos="0.3" ctrl="0"/></keyframe>
            </mujoco>
        """)

    def invoke(self, *args):
        with (
            patch("sys.argv", ["openarm-mujoco-launch", "test.xml", *args]),
            patch.object(launch.os.path, "exists", return_value=True),
            patch.object(launch.mujoco, "MjModel") as model_type,
        ):
            model_type.from_xml_path.return_value = self.model
            return launch.main()

    def test_dynamic_uses_managed_viewer_with_initialized_control(self):
        with (
            patch.object(launch.mujoco.viewer, "launch") as managed,
            patch.object(launch.mujoco.viewer, "launch_passive") as passive,
        ):
            self.assertEqual(self.invoke(), 0)
        managed.assert_called_once()
        passive.assert_not_called()
        model, data = managed.call_args.args
        self.assertIs(model, self.model)
        self.assertEqual(model.opt.timestep, 0.001)
        np.testing.assert_allclose(data.qpos, [0.3])
        np.testing.assert_allclose(data.ctrl, [0.3])

    def test_static_is_paced_without_integrating_or_following_ctrl(self):
        viewer = SimpleNamespace(
            cam=mujoco.MjvCamera(),
            lock=contextlib.nullcontext,
            is_running=MagicMock(side_effect=[True, True, False]),
            sync=MagicMock(),
        )
        with (
            patch.object(launch.mujoco.viewer, "launch") as managed,
            patch.object(launch.mujoco.viewer, "launch_passive") as passive,
            patch.object(launch.mujoco, "mj_step") as step,
            patch.object(launch.time, "perf_counter", side_effect=[0, 0.005, 1, 1.03]),
            patch.object(launch.time, "sleep") as sleep,
        ):
            passive.return_value.__enter__.return_value = viewer
            viewer.sync.side_effect = lambda: passive.call_args.args[1].ctrl.fill(1)
            self.assertEqual(self.invoke("--static"), 0)
        managed.assert_not_called()
        step.assert_not_called()
        self.assertEqual(viewer.sync.call_count, 2)
        sleep.assert_called_once_with(1 / 60 - 0.005)
        data = passive.call_args.args[1]
        self.assertEqual(data.time, 0)
        np.testing.assert_allclose(data.qpos, [0.3])
        np.testing.assert_allclose(data.ctrl, [1])

    def test_missing_keyframe_does_not_launch(self):
        with (
            patch.object(launch.mujoco.viewer, "launch") as managed,
            patch.object(launch.mujoco.viewer, "launch_passive") as passive,
            contextlib.redirect_stderr(io.StringIO()),
        ):
            self.assertEqual(self.invoke("--keyframe", "missing"), 2)
        managed.assert_not_called()
        passive.assert_not_called()


if __name__ == "__main__":
    unittest.main()
