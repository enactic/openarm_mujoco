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

import sysconfig
from pathlib import Path

from .joint_resolver import JointResolver as JointResolver


def _resolve_asset_path(root: Path, relative: str) -> Path:
    relative_path = Path(relative)
    if relative_path.is_absolute():
        msg = f"Asset path must be relative: {relative}"
        raise ValueError(msg)

    root = root.resolve()
    path = (root / relative_path).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        msg = f"Asset path must stay within asset root: {relative}"
        raise ValueError(msg) from exc

    return path


def _source_tree_asset_root() -> Path | None:
    current_file = Path(__file__).resolve()
    source_root = current_file.parent.parent.parent.parent
    source_package_file = source_root / "src" / "openarm_mujoco" / "v2" / "__init__.py"
    if current_file != source_package_file.resolve():
        return None

    asset_root = source_root / "v2"
    if asset_root.is_dir():
        return asset_root

    return None


def asset_path(relative: str) -> str:
    """Return an absolute filesystem path to a v2 MJCF asset.

    Example: asset_path("openarm_bimanual.xml")
    """
    source_tree_root = _source_tree_asset_root()
    if source_tree_root is not None:
        source_tree_path = _resolve_asset_path(source_tree_root, relative)
        if source_tree_path.exists():
            return str(source_tree_path)

    installed_root = (
        Path(sysconfig.get_path("data")) / "share" / "openarm_mujoco" / "v2"
    )
    return str(_resolve_asset_path(installed_root, relative))


def openarm_bimanual_paths() -> list[str]:
    """Return the list of the absolute path to bimanual file and the other required files/directories."""
    return [
        asset_path("openarm_bimanual.xml"),
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
    return asset_path("openarm_bimanual.xml")
