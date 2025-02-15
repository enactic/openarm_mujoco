# MuJoCo MJCF XML for OpenArm

https://github.com/user-attachments/assets/5452adfd-1496-4264-bd3d-56e6716dfa6f

## Improving Simulation Stability
- Short timestep of 0.001
- Runge Kutta integrator

## CAD to STL conversion
- Fasteners can cause collisions and constrain the range of movement of joints, so they have been removed in `J2_v1_1.stl`
- The `base_link` mesh is removed from collision detection by setting `contype` and `conaffinity` to zero, allowing the first DOF to rotate

## URDF to MJCF Conversion
- Grippers actuate equally using a tendon and equality constraint
