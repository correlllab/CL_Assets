#!/usr/bin/env python3
"""
Regenerates the perimeter screw rows in arpa_system.xml.

Edit the parameters in the USER-EDITABLE PARAMETERS section below, then run:

    python3 generate_screw_row.py

It rewrites everything between the sentinel comments
    <!-- SCREW_ROW_START -->  ...  <!-- SCREW_ROW_END -->
in arpa_system.xml, in place. Safe to re-run as many times as you want --
each run fully replaces the previous rows, it doesn't append.

Run it from the same directory as arpa_system.xml, or edit XML_PATH below.
"""
import math
import re

XML_PATH = "arpa_system.xml"

# ============================================================
# USER-EDITABLE PARAMETERS
# ============================================================
N_SCREWS = 18
SPACING_IN = 4.0                      # inches between adjacent screws

# Row anchor + direction, both expressed in battery_link's OWN local frame
# (the same frame the battery.stl mesh is defined in). z=0 is the mounting
# surface. screw_01 sits at ROW_ORIGIN_LOCAL; each subsequent screw steps
# SPACING_IN further along the rotated row direction.
ROW_ORIGIN_LOCAL = (0.603, -0.8625, 0.111)   # (x, y, z) meters
ROW_ANGLE_DEG = 90.0                         # extra in-plane rotation (deg) of the
                                              # row direction, added on top of
                                              # battery_link's own z-rotation

# Second (mirrored) row, about battery_link's local y-axis (x -> -x).
GENERATE_MIRROR_ROW = True

# If False: mirror row origin is auto-computed as (-x, y, z) of ROW_ORIGIN_LOCAL,
# and its direction/angle is mirrored to match (so the two rows are true
# reflections of each other about local x=0).
# If True: MIRROR_ORIGIN_LOCAL / MIRROR_ANGLE_DEG below are used verbatim
# instead -- use this once you've confirmed the real hole pattern isn't a
# perfect mirror of the first row.
MIRROR_OVERRIDE = False
MIRROR_ORIGIN_LOCAL = (-0.603, -0.8675, 0.111)
MIRROR_ANGLE_DEG = 180.0 - 90.0   # only used if MIRROR_OVERRIDE is True

# Thickness of the screw head's collision-proxy cylinder (meters). Independent
# of the real flange thickness (which only affects visual mesh placement) --
# tune this for contact stability without touching how the screw looks.
HEAD_COLLISION_THICKNESS = 0.005

# ============================================================
# FIXED CONSTANTS
# Derived from arpa_system.xml's cell geometry and the screw spec. Only
# touch these if the model's gantry/battery placement or the screw
# itself changes.
# ============================================================
IN = 0.0254

GANTRY_Z = 1.9419
BATTERY_POS = (0.067, -0.0235, -1.2119)   # battery_link pos, in gantry_link's frame
BATTERY_THETA = 1.57                    # battery_link euler about world z (radians)

# screw dims (meters)
FLANGE_R = 0.65 * IN / 2
FLANGE_T = 0.0015
HEX_T = (3 / 16) * IN
HEAD_H = FLANGE_T + HEX_T                 # total head stack height (flange+hex) --
                                           # also the geom pos offset for the visual mesh
SHAFT_R = 0.25 * IN / 2
SHAFT_L = 0.75 * IN
Z_SHAFT = -SHAFT_L / 2                    # shaft hangs below the mounting surface, into the hole
Z_FLANGE = HEAD_COLLISION_THICKNESS / 2   # collision proxy center height

HEAD_MASS = 0.00418                       # flange (2.03 g) + hex (2.15 g) combined -- carried
                                           # entirely by the flange collision-proxy cylinder now
SHAFT_MASS = 0.00382

# Head *visual* mesh orientation: verified empirically against screw_head.stl --
# do not change unless the STL itself is re-exported with a different origin/
# axis convention. euler="1.5707963 0 0" (90 deg about local x) + pos z = HEAD_H
# puts the flange-bottom contact face at local z=0 and the hex top at z=HEAD_H.
HEAD_EULER = "1.5707963 0 0"

# Contact params -- kept identical on the flange proxy and the battery geom
# (equal geom "priority" mixes matched values as a no-op; mismatched values
# would get solmix-blended). solref timeconst raised from an earlier 0.0005
# (unstable -- stiffer than 2x the 0.002s timestep) to 0.004.
CONTACT = ('friction="1 0.005 0.001" solref="0.004 1" '
           'solimp="0.95 0.99 0.001" priority="1"')


def battery_world_origin():
    return (BATTERY_POS[0], BATTERY_POS[1], GANTRY_Z + BATTERY_POS[2])


def local_to_world(lx, ly, lz, bx, by, bz, Bc, Bs):
    wx = bx + (lx * Bc - ly * Bs)
    wy = by + (lx * Bs + ly * Bc)
    wz = bz + lz
    return wx, wy, wz


def gen_row(origin_local, angle_deg, start_index, name_prefix="screw"):
    bx, by, bz = battery_world_origin()
    Bc, Bs = math.cos(BATTERY_THETA), math.sin(BATTERY_THETA)

    theta_total = BATTERY_THETA + math.radians(angle_deg)
    dc, ds = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))

    ox, oy, oz = origin_local
    blocks = []
    for i in range(N_SCREWS):
        d = i * SPACING_IN * IN
        lx = ox + d * dc
        ly = oy + d * ds
        lz = oz

        wx, wy, wz = local_to_world(lx, ly, lz, bx, by, bz, Bc, Bs)

        idx = f"{start_index + i + 1:02d}"
        blocks.append(f'''    <body name="{name_prefix}_{idx}" pos="{wx:.6f} {wy:.6f} {wz:.6f}" euler="0 0 {theta_total:.6f}">
      <joint name="{name_prefix}_{idx}_free" type="free"/>
      <geom name="{name_prefix}_{idx}_head_visual" type="mesh" mesh="screw_head" euler="{HEAD_EULER}" pos="0 0 {HEAD_H + HEAD_COLLISION_THICKNESS:.6f}"
            material="screw_steel" contype="0" conaffinity="0" density="0"/>
      <geom name="{name_prefix}_{idx}_head_collision" type="cylinder" size="{FLANGE_R:.6f} {HEAD_COLLISION_THICKNESS / 2:.6f}" pos="0 0 {Z_FLANGE:.6f}" group="3"
            mass="{HEAD_MASS}" material="screw_steel" {CONTACT}/>
      <geom name="{name_prefix}_{idx}_shaft" type="cylinder" size="{SHAFT_R:.6f} {SHAFT_L / 2:.6f}" pos="0 0 {Z_SHAFT:.6f}"
            mass="{SHAFT_MASS}" material="screw_steel" contype="0" conaffinity="0"/>
    </body>''')
    return blocks


def gen_all_rows():
    blocks = gen_row(ROW_ORIGIN_LOCAL, ROW_ANGLE_DEG, start_index=0)

    if GENERATE_MIRROR_ROW:
        if MIRROR_OVERRIDE:
            m_origin = MIRROR_ORIGIN_LOCAL
            m_angle = MIRROR_ANGLE_DEG
        else:
            ox, oy, oz = ROW_ORIGIN_LOCAL
            m_origin = (-ox, oy, oz)
            m_angle = 180.0 - ROW_ANGLE_DEG   # mirror the row direction about local x=0 too

        blocks += gen_row(m_origin, m_angle, start_index=N_SCREWS, name_prefix="screw")

    return "\n".join(blocks)


def main():
    with open(XML_PATH) as f:
        xml = f.read()

    pattern = re.compile(r"(<!-- SCREW_ROW_START -->)(.*?)(<!-- SCREW_ROW_END -->)", re.DOTALL)
    if not pattern.search(xml):
        raise RuntimeError(f"Sentinel comments not found in {XML_PATH} -- "
                            "expected <!-- SCREW_ROW_START --> / <!-- SCREW_ROW_END -->")

    new_block = gen_all_rows()
    xml = pattern.sub(lambda m: m.group(1) + "\n" + new_block + "\n    " + m.group(3), xml)

    with open(XML_PATH, "w") as f:
        f.write(xml)

    total = N_SCREWS * (2 if GENERATE_MIRROR_ROW else 1)
    print(f"Regenerated {total} screws in {XML_PATH} "
          f"({N_SCREWS} per row, mirror {'ON' if GENERATE_MIRROR_ROW else 'OFF'}, "
          f"mirror_override={MIRROR_OVERRIDE})")


if __name__ == "__main__":
    main()