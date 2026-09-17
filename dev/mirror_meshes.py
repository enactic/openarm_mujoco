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

"""Generate the pre-mirrored V2 collision meshes across Y=0."""

from pathlib import Path
import struct


_TRIANGLE = struct.Struct("<12fH")
_COLLISION = Path(__file__).resolve().parents[1] / "v2" / "assets" / "collision"
_MESHES = (
    ["base_link.stl", "ee_base_link.stl"]
    + [f"link{i}.stl" for i in (1, 2, 6)]
    + [f"link5_part_{i:02d}.stl" for i in range(3)]
    + [
        f"finger_{side}_part_{part:02d}.stl"
        for side in ("inner", "outer")
        for part in range(4)
    ]
)


def mirror_stl(source: Path, destination: Path) -> None:
    """Reflect binary STL vertices and normals, preserving outward winding."""
    data = source.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL header: {source}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * _TRIANGLE.size:
        raise ValueError(f"Invalid binary STL triangle count: {source}")
    output = bytearray(data)
    for offset in range(84, len(data), _TRIANGLE.size):
        values = list(_TRIANGLE.unpack_from(data, offset))
        for y in (1, 4, 7, 10):
            values[y] = -values[y]
        # Reflection changes handedness; swap two vertices of each triangle.
        values[6:9], values[9:12] = values[9:12], values[6:9]
        _TRIANGLE.pack_into(output, offset, *values)
    destination.write_bytes(output)


def main() -> None:
    """Regenerate the 16 mirrored V2 collision assets from their originals."""
    for name in _MESHES:
        source = _COLLISION / name
        mirror_stl(source, source.with_stem(f"{source.stem}_left"))


if __name__ == "__main__":
    main()
