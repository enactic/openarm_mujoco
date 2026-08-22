# MuJoCo Description Files (MJCF) for OpenArm
<img height="546" alt="image" src="media/v2.png" />

This repository contains assets for OpenArm v2 (above), Cell, v1 and v0.3 (below) simulation in MuJoCo.

## Usage

Install openarm-mujoco:

```bash
pip install openarm-mujoco
```

Launch the simulation:

```bash
openarm-mujoco-launch
```

Without White Sheet:

```bash
openarm-mujoco-launch --no-sheet
```

With Wall Collisions:

```bash
openarm-mujoco-launch --walls
```

## OpenArm MuJoCo Web

[OpenArm MuJoCo Web](web/README.md) runs these models right in the
browser (MuJoCo WASM + three.js, no installation): open
<https://enactic.github.io/openarm_mujoco/>, or serve the repository
root locally (`npm run serve` in `web/`) and open
<http://localhost:8080/>. Drive the arms by keyboard or sliders; see
[web/README.md](web/README.md) for details.

## Collision Visualization
- To view collision meshes, activate `Rendering`>`Model Elements`>`Convex Hull` and `Group Enable`>`Geom groups`>`Geom 3` in the left sidebar
- It may also help to hide the visual meshes by deselecting `Geom 2`

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/simulation/mujoco)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## Pull request and preview

You can enable preview on your fork by the following:

1. Enable GitHub Pages on your fork:
   1. Open https://github.com/${YOUR_GITHUB_ACCOUNT}/openarm_mujoco/settings/pages
   2. Select "GitHub Actions" as "Source"
2. Accept publishing GitHub Pages from all branches on your fork:
   1. Open https://github.com/${YOUR_GITHUB_ACCOUNT}/openarm_mujoco/settings/environments
   2. Select the "github-pages" environment
   3. Change the default "Deployment branches and tags" rule:
      1. Press the "Edit" button
      2. Change the "Name pattern" to `*` from `main`

You can preview your changes at https://${YOUR_GITHUB_ACCOUNT}.github.io/openarm_mujoco/ .

## License

Licensed under the Apache License 2.0. See `LICENSE` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
