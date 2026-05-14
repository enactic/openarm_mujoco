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

"""Shared qpos layout helpers for the OpenArm bimanual MuJoCo model.

Indices are discovered at runtime by resolving joint names, so the code is
robust to changes in MJCF qpos ordering.

Driver convention (8 values per arm):
  values[0:7] = joints 1–7
  values[7]   = gripper (finger_joint1; finger_joint2 is equality-constrained)

16-element combined driver: right[0:8] + left[8:16].
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class _ArmLayout:
    arm_qpos: np.ndarray  # shape (7,) – qpos indices for joint1..joint7
    arm_dof: np.ndarray   # shape (7,) – dof indices for joint1..joint7
    finger_qpos: int      # qpos index for finger_joint1
    mirror_qpos: int      # qpos index for finger_joint2 (equality mirror)


def _resolve_arm(prefix: str, model: mujoco.MjModel) -> _ArmLayout:
    def qpos_of(name: str) -> int:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"Joint '{name}' not found in model")
        return int(model.jnt_qposadr[jid])

    def dof_of(name: str) -> int:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"Joint '{name}' not found in model")
        return int(model.jnt_dofadr[jid])

    return _ArmLayout(
        arm_qpos    = np.array([qpos_of(f"{prefix}joint{i}") for i in range(1, 8)], dtype=np.intp),
        arm_dof     = np.array([dof_of(f"{prefix}joint{i}") for i in range(1, 8)], dtype=np.intp),
        finger_qpos = qpos_of(f"{prefix}finger_joint1"),
        mirror_qpos = qpos_of(f"{prefix}finger_joint2"),
    )

def _resolve_arm_ctrl(arm: str, model: mujoco.MjModel) -> np.ndarray:
    """Return shape-(8,) ctrl index array for one arm's joints + finger."""
    def act_idx(name: str) -> int:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise ValueError(f"Actuator '{name}' not found in model")
        return aid

    return np.array(
        [act_idx(f"{arm}_joint{i}_ctrl") for i in range(1, 8)] + [act_idx(f"{arm}_finger1_ctrl")],
        dtype=np.intp,
    )


class JointResolver:
    """Precomputed qpos/ctrl ↔ driver index mapping for the OpenArm bimanual model.

    Built once from a MjModel by resolving joint and actuator names, so it
    works with any MJCF that follows the openarm_left_* / openarm_right_*
    naming convention.

    Usage::

        mapper = JointResolver(model)
        mapper.set_qpos(data.qpos, right_cmd, "right")   # direct joint write
        mapper.set_ctrl(data.ctrl, right_cmd, "right")   # actuator target write
        joints, gripper = mapper.get_driver(data.qpos, "right")
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        self._right = _resolve_arm("openarm_right_", model)
        self._left  = _resolve_arm("openarm_left_",  model)

        self._right_ctrl: np.ndarray = _resolve_arm_ctrl("right", model)
        self._left_ctrl:  np.ndarray = _resolve_arm_ctrl("left",  model)

    def set_qpos(self, qpos: np.ndarray, driver_position: np.ndarray, segment: str) -> np.ndarray:
        """Write driver values for one segment into qpos in-place.

        Args:
            qpos: Full nq-slot qpos array to update (modified in-place).
            driver_position: 8-element array (joints[0:7] + gripper[7]).
            segment: One of "right", "left".

        Returns:
            The same qpos array (modified in-place).
        """
        if segment == "right":
            qpos[self._right.arm_qpos]    = driver_position[:7]
            qpos[self._right.finger_qpos] = driver_position[7]
            qpos[self._right.mirror_qpos] = driver_position[7]
        elif segment == "left":
            qpos[self._left.arm_qpos]    = driver_position[:7]
            qpos[self._left.finger_qpos] = driver_position[7]
            qpos[self._left.mirror_qpos] = driver_position[7]
        else:
            raise ValueError(f"Invalid segment: {segment!r}")
        return qpos

    def arm_qpos_indices(self, segment: str) -> np.ndarray:
        """Return shape-(7,) qpos indices for arm joints 1..7."""
        if segment == "right":
            return self._right.arm_qpos.copy()
        if segment == "left":
            return self._left.arm_qpos.copy()
        raise ValueError(f"Invalid segment for arm indices: {segment!r}")

    def arm_dof_indices(self, segment: str) -> np.ndarray:
        """Return shape-(7,) dof indices for arm joints 1..7."""
        if segment == "right":
            return self._right.arm_dof.copy()
        if segment == "left":
            return self._left.arm_dof.copy()
        raise ValueError(f"Invalid segment for arm indices: {segment!r}")

    def set_ctrl(self, ctrl: np.ndarray, driver_position: np.ndarray, segment: str) -> np.ndarray:
        """Write driver values for one segment into data.ctrl in-place.

        Args:
            ctrl: Full nu-slot ctrl array to update (modified in-place).
            driver_position: 8-element array (joints[0:7] + gripper[7]).
            segment: One of "right", "left".

        Returns:
            The same ctrl array (modified in-place).
        """
        if segment == "right":
            ctrl[self._right_ctrl] = driver_position[:8]
        elif segment == "left":
            ctrl[self._left_ctrl] = driver_position[:8]
        else:
            raise ValueError(f"Invalid segment: {segment!r}")
        return ctrl

    def get_driver(self, qpos: np.ndarray, segment: str) -> tuple[np.ndarray, float | np.ndarray]:
        """Read driver values for one segment from qpos (does not mutate qpos).

        Args:
            qpos: Full nq-slot qpos array.
            segment: One of "right", "left", "bimanual".

        Returns:
            (joints, gripper) where joints is shape (7,) for a single arm or
            shape (14,) for "bimanual", and gripper is a scalar float or a
            length-2 array for "bimanual".
        """
        if segment == "right":
            return qpos[self._right.arm_qpos], qpos[self._right.finger_qpos]
        elif segment == "left":
            return qpos[self._left.arm_qpos], qpos[self._left.finger_qpos]
        elif segment == "bimanual":
            return (
                np.concatenate([qpos[self._right.arm_qpos], qpos[self._left.arm_qpos]]),
                np.array([qpos[self._right.finger_qpos], qpos[self._left.finger_qpos]]),
            )
        else:
            raise ValueError(f"Invalid segment: {segment!r}")
