# Manipulation Board Assets

Reusable MuJoCo components for a NIST peg-in-hole board and five peg shapes.
These assets contain appearance, collision geometry, reference frames, and peg
dynamics. They are not a complete robot scene or a manipulation controller.

## Files

| File | Purpose |
| --- | --- |
| `workcell.xml` | Empty environment component. A consumer can add a primitive table here in memory. |
| `manipulation_board_functional.xml` | Recommended board: exact functional holes and blind floors, with cosmetic engravings filled in collision geometry. |
| `manipulation_board.xml` | Detailed board: also preserves engraved recesses in collisions. |
| `manipulation_peg_cylinder.xml` | Dynamic cylindrical peg. |
| `manipulation_peg_hex.xml` | Dynamic hexagonal peg. |
| `manipulation_peg_l.xml` | Dynamic L peg, with compound collisions preserving the notch. |
| `manipulation_peg_rectangle.xml` | Dynamic rectangular peg. |
| `manipulation_peg_triangle.xml` | Dynamic triangular peg. |
| `manipulation_catalog.json` | Component paths, hole poses, matching rules, physical assumptions, and source metadata. Read this in your application, not MuJoCo. |

Load **one board mode** and instantiate only the pegs needed. Functional mode
uses 3,005 collision geoms versus 10,895 in detailed mode; both use the original
visual mesh. Board collision vertices are embedded in the XMLs. Source STLs and
peg collision OBJs remain under `CL_Assets/meshes/manipulation_net/`.

## Paths and Units

Choose one absolute asset root pointing to the **CL_Assets directory**. It need
not be a submodule or live beside your package. Every path in the catalog and
every XML `file` attribute is relative to that root, for example:

```text
mujoco_assets/manipulation_net/manipulation_board_functional.xml
meshes/manipulation_net/nist_atb_m1.stl
```

Do not resolve these paths relative to this README, the XML directory, a scene
YAML, or the shell working directory. Before passing component XML to MuJoCo,
resolve file-backed resources to absolute paths under the asset root, clear
component compiler `meshdir`/`texturedir`, and disable `strippath`. Reject absolute
input references or paths escaping the root. The supplied components are
standalone; your composer handles attachment. A direct `from_xml_path()` call
without resource normalization is not a portable loader for these components.

Use metres, kilograms, radians, and quaternions in **w, x, y, z** order. The
source STLs use centimetres, but prepared XMLs already apply the conversion:
**do not scale the components again**. Table box dimensions in a user config
should be full dimensions; convert to MuJoCo half-sizes when creating geoms.

## Geometry and Frames

- Board footprint: **0.32 x 0.25 m**; thickness: **0.025 m**.
- Board body `board`: origin at its bottom mounting plane, centred in XY.
- Hole entry: board-local Z = **0.025 m**; blind floor: Z = **0.003 m**.
- Insertion depth: **0.022 m**. The board has its own floor, so it can rest on a solid table.
- Hole frame local +Z points outward. Insert along local -Z.
- Each peg has body `peg`, free joint `freejoint`, and bottom-tip site `tip`.
- Peg body origin and tip coincide; shafts extend along local +Z and are **0.05 m** long.
- Prepared body/site orientations use explicit quaternions or identity.

Hole IDs are `<shape>_<size>`, with shapes `l`, `rectangle`, `triangle`, `hex`,
and `cylinder`, and size IDs `0`, `1`, `2`, `3`. These IDs identify the source
board's **tight-to-loose clearance rows**, not scaled pegs. A family uses the
same physical peg XML across all four rows. Read each hole quaternion: profiles
are not necessarily aligned with the board axes.

Useful catalog fields:

| Field | Meaning |
| --- | --- |
| `version` | Catalog schema version, currently `1`. |
| `board.collision_xmls.functional` / `.detailed` | Select the board XML. `board.xml` is the detailed asset, not the fast default. |
| `board.footprint`, `.thickness`, `.floor_z`, `.entry_z` | Physical dimensions in metres. |
| `holes[hole_id]` | `shape`, integer `size`, `entry`, `target`, `quat`, `entry_site`, `target_site`, maximum and seated insertion depths. |
| `variants[shape][str(size)]` | Peg XML, body/joint/site names, dimensions, mass/inertia, and source transform. JSON size keys are strings. |
| `physical_assumptions`, `board.source`, variant `source` | Contact assumptions, source paths/hashes, and geometry provenance. |

For example, raw board sites for `cylinder_1` are `hole_cylinder_1_entry` and
`hole_cylinder_1_target`. The raw peg site is `tip`. Names change when your
composer applies prefixes.

## Build a Composer

Keep robot assets, object components, and user layout configuration separate.
Use MuJoCo's `MjSpec` attachment API; the reference consumer uses MuJoCo 3.6.

1. Read your YAML/JSON layout and the catalog. Validate unique instance IDs,
   occupied holes, matching shape/size pairs, and depths between zero and the
   hole's maximum depth. An omitted depth can default to `seated_depth`.
2. Load the complete robot scene as the parent spec. Preserve its defaults,
   actuators, sensors, loop constraints, and existing environment. Normalize its
   resources according to its own conventions, independently of these components.
3. Load `workcell.xml`, or create an empty spec. Add a fixed table from boxes.
   Define table height as its upper surface, then create a tabletop frame there.
4. Load the selected board with normalized resource paths. Attach its fixed body
   at the configured tabletop-relative pose. Do not add a free joint to the board.
5. For each enabled peg, load a **fresh** peg spec. Compute its initial world pose
   using the hole frame and desired depth. Keep the free peg at world level, not
   as a child of the table or board.
6. Attach the workcell and pegs to the robot parent with unique prefixes, using
   `parent.attach(child, frame=frame, prefix=...)`. Read component poses before
   attachment, which normally transfers their ownership. Avoid collisions with
   robot names; for example, `task_` and `task_peg_cylinder_1_`.
7. Set simulation options, compile once, then create `MjData` and run forward
   kinematics. Set poses before compilation so `model.qpos0` captures the layout;
   normal data reset can then restore it. Do not teleport pegs every step.

Pose math, using rigid transforms (not component-wise quaternion addition):

```text
T_world_board = T_world_table * T_table_tabletop * T_tabletop_board
T_world_hole = T_world_board * T_board_hole_entry
T_world_tip = T_world_hole * Translate(0, 0, -insertion_depth)
T_world_peg = T_world_tip * inverse(T_peg_tip)
```

The current `tip` transform is identity, but keep the final term if implementing
a reusable composer. Hole `entry`/`quat` describe the board-local pose; apply the
board pose before initializing the world-level free joint.

A minimal **layout example** for a consumer to interpret:

```yaml
board:
  catalog: mujoco_assets/manipulation_net/manipulation_catalog.json
  collision: functional
  position_on_table: [0.0, 0.0, 0.0]
  quaternion: [1.0, 0.0, 0.0, 0.0]
  pegs:
    - id: cylinder_1
      shape: cylinder
      size: 1
      hole: cylinder_1
      # Omit for seated; use 0.0 to put the bottom tip at the entry plane.
      insertion_depth: 0.022
```

This is not a complete runnable scene or a universal YAML schema. The reference
implementation is `h1_mujoco/scene_builder.py`; its local `scene/` directory has
complete layouts. Other packages can implement the same asset contract without
depending on that repository or its pelvis/table placement choices.

## Regeneration and Checks

The board/peg XMLs and catalog are generated. Modify the source meshes or the
preparation script instead of hand-editing generated output. `workcell.xml` and
this README are hand-maintained and are not overwritten by regeneration.

From the reference `h1_mujoco` checkout:

```bash
uv run --group dev utility/prepare_manipulation_assets.py          # regenerate
uv run --group dev utility/prepare_manipulation_assets.py --check  # verify only
uv run --group dev pytest tests/test_manipulation_paths.py -q      # file-only checks
```

The generator writes component files here and collision OBJs under
`meshes/manipulation_net/collision/`; original STLs are never rewritten.
The `--check` command recomputes geometry offline, checks generated contents,
and does not start a simulator or connect to a robot.

For a new consumer, test initial poses, reset, name uniqueness, preservation of
robot joint/actuator/sensor addresses, aligned insertion/withdrawal, offset wall
contact, and blind-floor contact. Run physics validation in an isolated offline
process, not alongside a live robot control loop.

Do not replace the board with one collidable mesh: convex-hull collision can
close its holes. Tight fits have micrometre-scale clearance. Density, friction,
and soft-contact settings are assumptions, not calibrated force predictions.
Grasp TCP, fingertip, and peg-tip frames are different; this asset pack does not
provide robot-specific grasp transforms, IK, or collision-free motion planning.
