# CL_Assets
A directory of assets H12 assets for the correll lab

## `mujoco_assets/battery/` — battery workcell (RoboCasa sim `--task Battery`)

- `battery_cell.xml` — the arena: ARPA cell, battery pack, a row of 36
  free-joint screws, walls, floor, lights. No robot; `h1_robocasa/battery_env.py`
  builds it around the robosuite H1-2. Loads standalone in `view_mjcf.py`.
- `meshes/` — battery and screw-head meshes used by the arena.
- `objects/*.xml` — parts placed in the scene. Every file here is merged into
  the task, so adding a part means adding a file.

### Adding an object

Write a standalone MJCF that opens on its own in `view_mjcf.py`, with one
top-level body per part (give it a `<freejoint>` to make it loose). Examples:
`objects/orange_fuse_block.xml`, `objects/hv_busbar_a01.xml`.

- The body `pos` is its world spawn pose. Battery module tops are at
  z = 0.932 m, the screw-row walls at 0.91 m and the channels between
  modules at 0.822 m.
- Only the top-level bodies and the assets they use are merged. A floor,
  lights and `<option>` for viewing the file on its own can stay; the task
  skips them.
- Degree or radian angles and `<default>` classes both work. Mesh and texture
  `file`s resolve against the file's own `meshdir` / `texturedir`.
- Mass and inertia are taken from the file compiled on its own, so the part
  weighs what it does in `view_mjcf.py`.
- The sim's viewer and cameras draw geom groups 1-2 only; a geom with no
  `group` is group 0 and invisible there. Put visible geoms in group 2 (they
  still collide), as `hv_busbar_a01.xml` does through its default class.
- Names (bodies, geoms, joints, materials, meshes) must be unique across the
  whole scene.
- `objects/make_meshes.py` builds the fuse block's visual mesh.

## `g1/` — the Unitree G1's model (GOLEM's `golem_ros2_models` and the G1 sim)

The G1's model source for GOLEM, the counterpart of `meshes/h1_2` and
`ros_assets/` for the H1-2.

- `g1/g1_description/` — the 29-DoF G1 as URDF and MJCF, body-only
  (`g1_29dof_rev_1_0.*`, rubber hands) and with the Dex3-1 hands
  (`g1_29dof_with_hand_rev_1_0.*`), and `meshes/` (51 STL). `meshdir="meshes"`
  in the MJCFs and `meshes/<name>.STL` in the URDFs resolve inside this
  directory.
- `g1/dex3_1_description/` — the standalone Dex3-1 hands (`dex3_1_{l,r}.{urdf,xml}`,
  `scene.xml`, `meshes/` with 16 STL).
- `g1/README.md` — the provenance of these files, file by file.
- `g1/LICENSE.unitree_ros` — their licence (BSD-3).

Source: the `leprechaun_g1` repo at `3bee054`, which vendored
unitreerobotics/unitree_ros at `ccfc6fd`. The files are byte for byte
upstream's, except the Dex3 MJCFs (`g1/README.md` lists what changed). All
meshes are Git-LFS objects (`*.STL`).

### The G1's sphere model and SRDF

`g1/g1_sphere.urdf` and `g1/g1_sphere_collision.srdf` are the G1's
self-collision model, supplied by the lab and not generated in this repo (there
is no `exclude_pair` script for them). They sit beside `g1_description/`
because they are the lab's files, not Unitree's.

- `g1_sphere.urdf` packs 24 spheres (each as both collision and visual
  geometry, no meshes) onto the body-only G1, with its joints in `/lowcmd`
  slot order. It was packed on the rubber-hand G1: each hand is two spheres on a
  `{left,right}_rubber_hand` link at the Dex3 palm's mount, so the Dex3 fingers
  are not covered.
- `g1_sphere_collision.srdf` lists the collision pairs to skip. Unlike the
  H1-2's, it keeps the pelvis and hip-pitch spheres active against the torso,
  the arms and the hands.
