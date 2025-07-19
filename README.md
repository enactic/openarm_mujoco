# MuJoCo MJCF for OpenArm

<img width="514" alt="image" src="https://github.com/user-attachments/assets/84ed9338-8990-46d3-8aee-1e87c55583b5" />

## Usage

https://github.com/user-attachments/assets/814ee3e1-36dd-4561-a837-25e921838253

1. Install MuJoCo and launch a simulation.
2. Drag the `*.xml` file into the simulation window.

For a detailed installation guide, see: [github.com/enactic/openarm_simulation/blob/main/MuJoCo.md](https://github.com/enactic/openarm_simulation/blob/main/openarm_mujoco/README_MuJoCo.md)

## Improving Simulation Stability
- Short timestep of < 0.001
- Runge Kutta integrator
- Simplified collision geometry

## CAD to STL conversion
- Setting `contype` and `conaffinity` to zero uses a geom as a visual mesh
- To view collision bounding boxes, activate `Rendering`>`Model Elements`>`Body Tree` in the left sidebar
- Binary meshes (`*.stl`) are needed, so this conversion should be done in CAD or with [MeshLab](https://github.com/cnr-isti-vclab/meshlab)

## URDF to MJCF Conversion
- Grippers actuate equally using a tendon and equality constraint, which uses a mimic joint in URDF
- Off-diagonal inertias are set to zero

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
