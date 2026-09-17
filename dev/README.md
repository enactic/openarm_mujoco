# Development

## Mirrored collision meshes

V2 uses 16 pre-mirrored collision meshes as a temporary
workaround for [MuJoCo #3604](https://github.com/google-deepmind/mujoco/issues/3604).
Negative mesh scale can produce inward convex polygon normals and incorrect
contacts. The V2 arm and gripper collision assets load with positive scale.
Visual mesh definitions retain their original files and scales.

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
