# MuJoCo Description Files (MJCF) for OpenArm
<img height="546" alt="image" src="https://github.com/user-attachments/assets/b969665c-75f9-470d-b327-e3806ae66002" />

This repository contains assets for OpenArm v1 (above) and v0.3 (below) simulation in MuJoCo. 

## Usage

https://github.com/user-attachments/assets/814ee3e1-36dd-4561-a837-25e921838253

1. Install MuJoCo and launch a simulation.
2. Drag the `*.xml` file into the simulation window.

The motors use torque control, so position and velocity control can be achieved by implementing a simple PD controller in client code.

## Collision Visualization
- To view collision meshes, activate `Rendering`>`Model Elements`>`Convex Hull` and `Group Enable`>`Geom groups`>`Geom 3` in the left sidebar
- It may also help to hide the visual meshes by deselecting `Geom 2`

<img height="534" alt="image" src="https://github.com/user-attachments/assets/1b003bf4-f0ce-42b9-89df-c8efd073cde7" />

## ROS2 Hardware Interface

MuJoCo can be used to simulate realistic hardware interfaces to test low-frequency control without physical hardware. 

https://github.com/enactic/openarm_mujoco_hardware


## URDF to MJCF Conversion
- Grippers actuate equally using a tendon and equality constraint, which uses a mimic joint in URDF
- Off-diagonal inertias are set to zero
- Motors require `ctrlrange` limits and joints have additional runtime properties `damping` and `frictionloss`

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/simulation/mujoco)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
