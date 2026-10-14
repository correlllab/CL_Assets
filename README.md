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

Write a standalone MJCF (it should open on its own in `view_mjcf.py`) holding
only `<compiler>`, `<asset>` and `<worldbody>`, with one body and a
`<freejoint>`. See `objects/orange_fuse_block.xml`.

- The body `pos` is its world spawn pose. The battery top is at z = 0.91 m.
- Mesh and texture `file`s resolve against the file's own `meshdir` /
  `texturedir` (e.g. `objects/assets/`).
- Use `<compiler angle="radian"/>` if you use `euler` or `axisangle`.
- Mass must come from a group-0 geom (no `group` attribute) or an explicit
  `<inertial>`; the sim ignores mass on geoms in other groups.
- Group-0 geoms are collision-only and aren't drawn by the viewer or the
  cameras. Give the part a visual geom in group 2 with `contype="0"
  conaffinity="0" density="0"`.
- Names (bodies, geoms, joints, materials, meshes) must be unique across the
  whole scene.
