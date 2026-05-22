import re
from collections import defaultdict
from pathlib import Path

from setuptools import setup


_VERSION_DIR_PATTERN = re.compile(r"v\d+(?:\.\d+)*$")
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
    return sorted(
        path
        for path in Path(".").iterdir()
        if path.is_dir() and _VERSION_DIR_PATTERN.fullmatch(path.name)
    )


def _data_files() -> list[tuple[str, list[str]]]:
    files_by_target: dict[str, list[str]] = defaultdict(list)

    for root in _version_dirs():
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix.lower() not in _ALLOWED_SUFFIXES:
                continue
            if any(
                part.startswith(".") or part in _EXCLUDED_PARTS for part in path.parts
            ):
                continue

            target = Path("share") / "openarm_mujoco" / path.parent
            files_by_target[str(target)].append(str(path))

    return [
        (target, sorted(files)) for target, files in sorted(files_by_target.items())
    ]


setup(data_files=_data_files())
