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

from collections import defaultdict
from pathlib import Path

from setuptools import setup


_PROJECT_ROOT = Path(__file__).resolve().parent
_BUNDLED_VERSION_DIRS = ("v2",)
_EXCLUDED_PARTS = {
    ".venv",
    "__pycache__",
    "openarm_mujoco.egg-info",
    "openarm_mujoco_v2",
}
_ALLOWED_SUFFIXES = {
    ".jpeg",
    ".jpg",
    ".mjcf",
    ".mtl",
    ".obj",
    ".png",
    ".stl",
    ".xml",
}


def _version_dirs() -> list[Path]:
    return [
        path
        for version_dir in _BUNDLED_VERSION_DIRS
        if (path := _PROJECT_ROOT / version_dir).is_dir()
    ]


def _data_files() -> list[tuple[str, list[str]]]:
    files_by_target: dict[str, list[str]] = defaultdict(list)

    for root in _version_dirs():
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix.lower() not in _ALLOWED_SUFFIXES:
                continue
            relative_path = path.relative_to(_PROJECT_ROOT)
            if any(
                part.startswith(".") or part in _EXCLUDED_PARTS
                for part in relative_path.parts
            ):
                continue

            target = Path("share") / "openarm_mujoco" / relative_path.parent
            files_by_target[str(target)].append(str(relative_path))

    return [
        (target, sorted(files)) for target, files in sorted(files_by_target.items())
    ]


setup(data_files=_data_files())
