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

"""Provide MuJoCo related files and convenient utilities."""

from .joint_resolver import JointResolver as JointResolver


def asset_path(relative: str) -> str:
    """Return an absolute filesystem path to a v2 MJCF asset.

    Example: asset_path("openarm_bimanual.xml")
    """
    import sysconfig
    from pathlib import Path

    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        source_tree_path = parent / "v2" / relative
        if source_tree_path.exists():
            return str(source_tree_path)

    installed_path = (
        Path(sysconfig.get_path("data")) / "share" / "openarm_mujoco" / "v2" / relative
    )
    return str(installed_path)


def openarm_bimanual_paths() -> list[str]:
    """Return the list of the absolute path to bimanual file and the other required files/directories."""
    return [
        asset_path("openarm_v20_bimanual.xml"),
        asset_path("assets"),
    ]


def openarm_cell_xml() -> str:
    """Return the XML path for OpenArm in OpenArm Cell."""
    return asset_path("cell.xml")


def openarm_demo_xml() -> str:
    """Return the XML path for OpenArm Cell demo."""
    return asset_path("demo.xml")


def openarm_pedestal_xml() -> str:
    """Return the XML path with pedestal."""
    return asset_path("pedestal.xml")


def openarm_bimanual_xml() -> str:
    """Return the XML path for bimanual OpenArm."""
    return asset_path("openarm_v20_bimanual.xml")
