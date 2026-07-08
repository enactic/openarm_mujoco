# Arm + Pedestal Component Standardization (v2)

Date: 2026-07-08
Status: Approved

## Problem

All 8 v2 scene files (`pedestal.xml` and the 7 task scenes: `valve_scene.xml`,
`articulated_scene.xml`, `bottle_scene.xml`, `balance_scene.xml`,
`move_puck_scene.xml`, `tool_scene.xml`, `peg_socket_scene.xml`) duplicate the
same boilerplate to place the OpenArm bimanual arms on the pedestal:

- material `openarm_body_link0_material`
- meshes `body_link0` (visual) and `body_link0_symp` (collision)
- the two pedestal geoms in worldbody
- `<frame pos="0 0 0.698"><attach model="bimanual" prefix=""/></frame>`

Any change to the pedestal or mount height must be repeated in 8 places.

## Decision

Create a new attachable component `v2/openarm_pedestal.xml` (pedestal +
bimanual arms) and have every scene attach it, mirroring the proven nested
attach chain `demo.xml -> cell.xml -> openarm_bimanual.xml`.

### Alternatives considered

- Making `pedestal.xml` itself the attachable component: rejected — it carries
  floor/light/keyframe, which would duplicate the floor and clash keyframe
  names when attached; stripping it would degrade standalone viewing, which
  the Python package exposes (`openarm_mujoco.v2` returns its path).
- `<include>`-based snippets: rejected — position-dependent textual splicing,
  no encapsulation, splits into multiple fragment files.

## Design

1. **New `v2/openarm_pedestal.xml`** — attachable component containing:
   - `<compiler meshdir="assets" angle="radian"/>`
   - default classes `pedestal_visual` / `pedestal_collision` (named to avoid
     colliding with scene-level `visual`/`collision`, following the
     `cell_visual`/`cell_collision` convention)
   - assets: pedestal material, the two `body_link0` meshes, and
     `<model name="bimanual" file="openarm_bimanual.xml"/>`
   - worldbody: the two pedestal geoms and the z=0.698 frame attaching the
     bimanual model with `prefix=""`
   - No floor, lights, cameras, statistic, option, or keyframes.
2. **`pedestal.xml`** becomes a thin standalone viewer scene: floor, light,
   skybox, statistic, visual settings, keyframe `home`, and an attach of the
   component. Filename, model name, and Python package usage unchanged.
3. **Task scenes** replace the boilerplate with
   `<model name="arm_pedestal" file="openarm_pedestal.xml"/>` +
   `<attach model="arm_pedestal" prefix=""/>` placed where the old pedestal
   geoms/frame were (before object bodies, so joint/qpos ordering is
   preserved). Scene-specific floors, cameras, objects, welds, and keyframes
   stay in the scenes. Keyframes must stay scene-side because each scene's nq
   includes its own object DOFs.
4. **Default class cleanup**: scene-level `visual`/`collision` classes that
   become unused are removed. `bottle_scene.xml` keeps `visual` (used by its
   own scene objects).

## Verification

For each scene, compile the git-HEAD version and the converted version with
MuJoCo (3.8.1) and assert identical: nq/nv/nu/nbody/ngeom/njnt, body/geom/
joint/actuator names and positions, equality constraints, and keyframes.
Also compile `openarm_pedestal.xml` standalone. Then `pre-commit run
--all-files`.
