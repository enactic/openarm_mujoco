# MuJoCo MJCF for OpenArm

https://github.com/user-attachments/assets/5452adfd-1496-4264-bd3d-56e6716dfa6f

## Usage

1. Install MuJoCo and launch a simulation.
2. Drag the `*.xml` file into the simulation window.

For a detailed installation guide, see: [github.com/reazon-research/openarm-sim/blob/main/MuJoCo.md](https://github.com/reazon-research/openarm-sim/blob/main/MuJoCo.md)

## Improving Simulation Stability
- Short timestep of 0.001
- Runge Kutta integrator

## CAD to STL conversion
- Fasteners can cause collisions and constrain the range of movement of joints, so they have been removed in `J2_v1_1.stl`
- The `base_link` mesh is removed from collision detection by setting `contype` and `conaffinity` to zero, allowing the first DOF to rotate

## URDF to MJCF Conversion
- Grippers actuate equally using a tendon and equality constraint
