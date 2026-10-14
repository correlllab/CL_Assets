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
