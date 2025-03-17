# MuJoCo MJCF for OpenArm

https://github.com/user-attachments/assets/5452adfd-1496-4264-bd3d-56e6716dfa6f

## Usage

1. Install MuJoCo and launch a simulation.
2. Drag the `*.xml` file into the simulation window.

For a detailed installation guide, see: [github.com/reazon-research/openarm-sim/blob/main/MuJoCo.md](https://github.com/reazon-research/openarm-sim/blob/main/openarm_mujoco/README_MuJoCo.md)

## Improving Simulation Stability
- Short timestep of < 0.001
- Runge Kutta integrator
- Simplified collision geometry

## CAD to STL conversion
- Setting `contype` and `conaffinity` to zero uses a geom as a visual mesh.
- To view collision bounding boxex, activate `Rendering`>`Model Elements`>`Body Tree` in the left sidebar.

## URDF to MJCF Conversion
- Grippers actuate equally using a tendon and equality constraint, which uses a mimic joint in URDF
