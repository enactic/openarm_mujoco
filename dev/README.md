# Development

## Mirrored collision meshes

V1 uses six pre-mirrored arm collision meshes, and V2 uses 16 pre-mirrored
arm and gripper collision meshes, as a temporary
workaround for [MuJoCo #3604](https://github.com/google-deepmind/mujoco/issues/3604).
Negative mesh scale can produce inward convex polygon normals and incorrect
contacts. All V1/V2 collision assets load with positive scale.
Arm visual mesh definitions retain their original files and scales.

V1 fingers share `finger.stl`; the opposing finger geometry is rotated 180
degrees around Z instead of using a mirrored mesh.

Regenerate the `_left` assets after updating their original meshes:

```bash
python dev/mirror_meshes.py
```

The script reflects vertices and normals across Y=0 and reverses triangle
winding. Joint/control parameters retain their existing definitions.

## How to release

```bash
git clone git@github.com:enactic/openarm_mujoco.git
cd openarm_mujoco
dev/release.sh ${VERSION} # e.g. dev/release.sh 1.0.0
```
