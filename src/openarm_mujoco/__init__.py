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
    """Return an absolute filesystem path to an asset inside this package.

    Example: asset_path("openarm_bimanual.xml")
    """
    from importlib.resources import files
    from pathlib import Path

    # src/openarm_mujoco/__init__.py ->
    # src/openarm_mujoco ->
    # src ->
    # . ->
    # ./v2
    v2_path = Path(__file__).parent.parent.parent / "v2"
    if v2_path.exists():  # For editable install
        return str(v2_path / relative)
    else:
        p = files("openarm_mujoco.v2").joinpath(relative)
        return str(Path(p))


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
