#!/usr/bin/env python3
"""Generate robosuite-conformant H1-2 + Magpie assets from the hand-written sim MJCF.

Source of truth is ``CL_Assets/mujoco_assets/h1_2_magpie.xml`` (the single merged
MJCF the legacy custom loop loads). robosuite's robot/gripper loaders impose
conventions that file violates, so this script mechanically transforms it into:

  robosuite_assets/robots/h1_2/robot.xml          # full-legged, single root body
  robosuite_assets/grippers/magpie_right_gripper.xml
  robosuite_assets/grippers/magpie_left_gripper.xml

Transforms applied (see docs/superpowers/specs/2026-06-08-...-design.md §7.3):
  * free joint  <joint type="free">  ->  <freejoint/>  (kept; full-legged base)
  * leg joints renamed to contain the substring "leg" so robosuite's
    substring part-classification groups them as legs, not arms. The ROS-facing
    names are unchanged; h1_mujoco/h1_2_robosuite.py keeps a ROS<->sim translation
    table (LEG_JOINT_RENAME, re-exported below).
  * grippers extracted into standalone GripperModel MJCFs with the
    robosuite-mandated `eef` body and `grip_site`/`grip_site_cylinder` sites
    (invisible, group 4) and the right gripper's two actuated joints renamed to
    finger_joint1/finger_joint2 (RoboCasa check_obj_grasped hard-codes
    gripper0_right_finger_joint1/2).
  * nested <default>/childclass flattened to per-element attrs (robosuite's stock
    MujocoXML loader chokes on nested classes).
  * <compiler meshdir> dropped; mesh file= refs rewritten relative to the new
    asset location so both raw MuJoCo and robosuite resolve them.
  * floor / lights / skybox / <visual> stripped (those belong to the arena).

Run from anywhere:  python CL_Assets/robosuite_assets/build_assets.py
It compiles each output with mujoco at the end as a smoke test.
"""
import copy
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
CL_ASSETS = os.path.normpath(os.path.join(HERE, ".."))
SRC = os.path.join(CL_ASSETS, "mujoco_assets", "h1_2_magpie.xml")
# Each side has its own source fragment. They are geometrically identical; the
# left one carries a "leftg_" name prefix (so both can coexist in the legacy
# combined h1_2_magpie.xml). build_gripper strips that prefix for the left so
# both outputs share the canonical bare names robosuite/RoboCasa expect.
GRIPPER_FRAG_R = os.path.join(CL_ASSETS, "mujoco_assets", "magpie_gripper.xml")
GRIPPER_FRAG_L = os.path.join(CL_ASSETS, "mujoco_assets", "magpie_gripper_left.xml")

OUT_ROBOT = os.path.join(HERE, "robots", "h1_2", "robot.xml")
OUT_GRIP_R = os.path.join(HERE, "grippers", "magpie_right_gripper.xml")
OUT_GRIP_L = os.path.join(HERE, "grippers", "magpie_left_gripper.xml")

# Mesh dir is CL_Assets/meshes/{h1_2,magpie}. Paths are written relative to each
# output XML's directory so robosuite (which joins file= with the XML dir and
# ignores <compiler meshdir>) and raw MuJoCo both resolve them.
ROBOT_MESH_PREFIX = "../../../meshes"      # from robosuite_assets/robots/h1_2/
GRIPPER_MESH_PREFIX = "../../meshes"       # from robosuite_assets/grippers/

# ROS joint name -> robosuite/sim joint name. Only the 12 legs change; arms/torso
# keep their names. Re-exported and consumed by the name-based DDS bridge.
LEG_JOINT_RENAME = {}
for _side in ("left", "right"):
    for _j in ("hip_yaw", "hip_pitch", "hip_roll", "knee", "ankle_pitch", "ankle_roll"):
        LEG_JOINT_RENAME[f"{_side}_{_j}_joint"] = f"{_side}_leg_{_j}_joint"

# The right gripper's two ACTUATED crank joints -> robosuite/RoboCasa finger joints.
FINGER_JOINT_RENAME = {
    "left_hinge_1": "finger_joint1",
    "right_hinge_1": "finger_joint2",
}


def build_flat_defaults(default_elem):
    """Flatten a nested <default> tree into {class_name: {tag: {attr: val}}}
    with full parent->child inheritance, mirroring MuJoCo's resolution."""
    flat = {}

    def walk(node, inherited):
        own = {}
        for child in node:
            if child.tag == "default":
                continue
            own.setdefault(child.tag, {}).update(child.attrib)
        merged = {tag: dict(inherited.get(tag, {})) for tag in set(inherited) | set(own)}
        for tag, attrs in own.items():
            merged.setdefault(tag, {}).update(attrs)
        cls = node.get("class")
        if cls is not None:
            flat[cls] = merged
        for child in node:
            if child.tag == "default":
                walk(child, merged)

    for child in default_elem:
        if child.tag == "default":
            walk(child, {})
    return flat


def inline_class(elem, flat):
    """Replace an element's class= reference with the resolved default attrs
    (element's own attrs win)."""
    cls = elem.attrib.pop("class", None)
    if cls and cls in flat and elem.tag in flat[cls]:
        for k, v in flat[cls][elem.tag].items():
            elem.attrib.setdefault(k, v)


def apply_joint_dynamics(joint, attrs):
    for k, v in attrs.items():
        joint.attrib.setdefault(k, v)


# robosuite's _add_default_name_filter auto-names unnamed geoms via
# group_mapping = {None:"col", "0":"col", "1":"vis"} and KeyErrors on any other
# group. Our source uses the legacy MuJoCo-viewer convention (visual=2,
# collision=3). Remap to robosuite's {0=col, 1=vis}.
_GEOM_GROUP_REMAP = {"2": "1", "3": "0"}


def remap_geom_groups(root):
    for g in root.iter("geom"):
        grp = g.get("group")
        if grp in _GEOM_GROUP_REMAP:
            g.set("group", _GEOM_GROUP_REMAP[grp])
        # Visual geoms (final group "1") must NEVER collide: force contype=0/conaffinity=0
        # on every group="1" geom -- those copied from the source AND ones emitted by this
        # script (e.g. the h12_mount visual mesh) -- so visual meshes can't hit the floor
        # regardless of the source's attrs. A missing contype=0 on the ankle/foot visuals
        # was the FAME-stand root cause (source regression CL_Assets 83d9bf4); enforcing it
        # here keeps the generated robot.xml correct even if the source regresses again.
        # Collision geoms (group "0") keep their source contype/conaffinity untouched.
        if g.get("group") == "1":
            g.set("contype", "0")
            g.set("conaffinity", "0")


# --------------------------------------------------------------------------- #
# Load source
# --------------------------------------------------------------------------- #
src_tree = ET.parse(SRC)
src_root = src_tree.getroot()
flat = build_flat_defaults(src_root.find("default"))
H12_JOINT_DYN = flat["h1_2"]["joint"]          # damping/armature/frictionloss


def rewrite_mesh_files(asset_elem, prefix):
    for mesh in asset_elem.findall("mesh"):
        f = mesh.get("file")
        if f:
            mesh.set("file", f"{prefix}/{f}")


# --------------------------------------------------------------------------- #
# robot.xml
# --------------------------------------------------------------------------- #
def build_robot():
    mj = ET.Element("mujoco", {"model": "h1_2"})
    ET.SubElement(mj, "compiler", {"angle": "radian", "autolimits": "true"})

    # asset: h1_2 meshes + the h12 wrist-mount adapter mesh only (gripper meshes
    # live in the gripper models now).
    asset = ET.SubElement(mj, "asset")
    src_asset = src_root.find("asset")
    keep_meshes = set()
    for mesh in src_asset.findall("mesh"):
        f = mesh.get("file", "")
        if f.startswith("h1_2/") or mesh.get("name") == "h12_mount":
            asset.append(copy.deepcopy(mesh))
            keep_meshes.add(mesh.get("name"))
    rewrite_mesh_files(asset, ROBOT_MESH_PREFIX)

    # worldbody: the pelvis subtree only.
    worldbody = ET.SubElement(mj, "worldbody")
    pelvis = copy.deepcopy(src_root.find("worldbody").find("./body[@name='pelvis']"))
    pelvis.attrib.pop("childclass", None)
    worldbody.append(pelvis)

    # free joint: <joint type="free"> -> <freejoint/>
    for body in pelvis.iter("body"):
        pass
    for parent in pelvis.iter():
        for child in list(parent):
            if child.tag == "joint" and child.get("type") == "free":
                fj = ET.Element("freejoint", {"name": child.get("name", "floating_base_joint")})
                idx = list(parent).index(child)
                parent.remove(child)
                parent.insert(idx, fj)

    # joints: inline h1_2 dynamics, rename legs
    for joint in pelvis.iter("joint"):
        apply_joint_dynamics(joint, H12_JOINT_DYN)
        nm = joint.get("name")
        if nm in LEG_JOINT_RENAME:
            joint.set("name", LEG_JOINT_RENAME[nm])

    # inline any remaining class= on geoms (h12_collision uses class="collision")
    for geom in pelvis.iter("geom"):
        if "class" in geom.attrib:
            inline_class(geom, flat)

    # eef anchor bodies: turn the gripper_attachment bodies into empty bodies
    # named right_hand/left_hand (robosuite merges the gripper onto these).
    attach_rename = {"gripper_attachment": "right_hand", "gripper_attachment_L": "left_hand"}
    for parent in pelvis.iter():
        for body in list(parent.findall("body")):
            nm = body.get("name")
            if nm in attach_rename:
                for ch in list(body):
                    body.remove(ch)          # drop the <include>
                body.set("name", attach_rename[nm])

    # rename the existing wrist-tip sites so they don't collide with the new bodies
    for site in pelvis.iter("site"):
        if site.get("name") == "right_hand":
            site.set("name", "right_hand_tip")
        elif site.get("name") == "left_hand":
            site.set("name", "left_hand_tip")

    # robosuite's composite controller resolves a per-arm base site
    # "{prefix}{arm}_center" in update_state(); add them at each shoulder mount
    # (mirrors GR1's right_center/left_center placed at pos 0 0 0 in the arm_mount).
    arm_center = {"left_shoulder_pitch_link": "left_center",
                  "right_shoulder_pitch_link": "right_center"}
    for body in pelvis.iter("body"):
        sname = arm_center.get(body.get("name"))
        if sname:
            ET.SubElement(body, "site", {
                "name": sname, "pos": "0 0 0", "size": "0.01",
                "group": "1", "rgba": "1 0.3 0.3 1",
            })

    # actuator: 27 body motors (leg-renamed), no gripper actuators
    actuator = ET.SubElement(mj, "actuator")
    for motor in src_root.find("actuator").findall("motor"):
        m = copy.deepcopy(motor)
        for attr in ("name", "joint"):
            v = m.get(attr)
            if v in LEG_JOINT_RENAME:
                m.set(attr, LEG_JOINT_RENAME[v])
        actuator.append(m)

    # sensor: body sensors (leg-renamed); drop finger (hinge) sensors
    sensor = ET.SubElement(mj, "sensor")
    for s in src_root.find("sensor"):
        if "hinge" in (s.get("joint") or ""):
            continue
        s2 = copy.deepcopy(s)
        for attr in ("name", "joint"):
            v = s2.get(attr)
            if v in LEG_JOINT_RENAME:
                s2.set(attr, LEG_JOINT_RENAME[v])
        sensor.append(s2)

    # contact excludes
    contact_src = src_root.find("contact")
    if contact_src is not None:
        mj.append(copy.deepcopy(contact_src))

    return ET.ElementTree(mj)


# --------------------------------------------------------------------------- #
# gripper xml (one builder; emitted for both sides with identical internal names
# since robosuite namespaces them gripper0_right_/gripper0_left_)
# --------------------------------------------------------------------------- #
def build_gripper(model_name, frag_path, strip_prefix=None):
    frag = ET.parse(frag_path).getroot()         # root <body name="mount">
    mount = copy.deepcopy(frag)

    # The left fragment names everything "leftg_*" so it stays unique inside the
    # legacy combined model. robosuite emits each gripper as its own file and
    # re-prefixes with gripper0_<side>_, so the leftg_ prefix is redundant here
    # and breaks downstream code that hardcodes bare names (_important_geoms's
    # left_pad*/right_pad*, FINGER_JOINT_RENAME's left_hinge_1/right_hinge_1, the
    # bare 4-bar equality body refs). Strip it so the left output matches the
    # right's canonical names; the rest of this builder then applies unchanged.
    if strip_prefix:
        for el in mount.iter():
            for attr in ("name", "joint"):
                v = el.get(attr)
                if v and v.startswith(strip_prefix):
                    el.set(attr, v[len(strip_prefix):])

    # inline geom classes; set joint dynamics (preserve currently-inherited h1_2
    # joint values); rename actuated joints to finger_joint1/2
    for geom in mount.iter("geom"):
        if "class" in geom.attrib:
            inline_class(geom, flat)
    for joint in mount.iter("joint"):
        apply_joint_dynamics(joint, H12_JOINT_DYN)
        nm = joint.get("name")
        if nm in FINGER_JOINT_RENAME:
            joint.set("name", FINGER_JOINT_RENAME[nm])

    # robosuite hard-requires a body named "eef" (GripperModel.__init__ reads
    # its quat for rotation_offset) and sites named grip_site/grip_site_cylinder
    # (mobile_robot.setup_references resolves gripper.important_sites at env
    # construction; RoboCasa task metrics read grip_site's world position), so
    # none of these can be dropped. Keep them at the same closed-finger grasp
    # center as the robot-level *_grasp_frame sites but in group 4 (like
    # touch_/tip_) so they never show in the viewer or camera renders.
    eef = ET.SubElement(mount, "body", {"name": "eef", "pos": "-0.00375 0 0.152991"})
    ET.SubElement(eef, "site", {"name": "grip_site", "pos": "0 0 0", "size": "0.01",
                                "rgba": "1 0 0 0.5", "group": "4", "type": "sphere"})
    ET.SubElement(eef, "site", {"name": "grip_site_cylinder", "pos": "0 0 0",
                                "type": "cylinder", "size": "0.005 0.05",
                                "rgba": "0 1 0 0.3", "group": "4"})

    # EE force/torque frame. robosuite's robot.control() reads force_ee/torque_ee
    # (gripper.important_sensors) every control step, so the gripper MUST declare
    # them or reset()'s settling loop KeyErrors. We don't use the values.
    ET.SubElement(mount, "site", {"name": "ft_frame", "pos": "0 0 0", "size": "0.01",
                                  "group": "1", "rgba": "1 0 0 1"})

    mj = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(mj, "compiler", {"angle": "radian", "autolimits": "true"})

    # asset: magpie meshes used by the gripper bodies
    asset = ET.SubElement(mj, "asset")
    used = {g.get("mesh") for g in mount.iter("geom") if g.get("mesh")}
    for mesh in src_root.find("asset").findall("mesh"):
        if mesh.get("name") in used:
            asset.append(copy.deepcopy(mesh))
    rewrite_mesh_files(asset, GRIPPER_MESH_PREFIX)

    worldbody = ET.SubElement(mj, "worldbody")
    worldbody.append(mount)

    # equality: the two 4-bar closures for this gripper (right-side names)
    equality = ET.SubElement(mj, "equality")
    for con in src_root.find("equality").findall("connect"):
        if con.get("name") in ("left_4bar_close", "right_4bar_close"):
            equality.append(copy.deepcopy(con))

    # actuator: the two finger position actuators (renamed joints)
    actuator = ET.SubElement(mj, "actuator")
    for pos in src_root.find("actuator").findall("position"):
        j = pos.get("joint")
        if j in FINGER_JOINT_RENAME:
            p = copy.deepcopy(pos)
            p.set("joint", FINGER_JOINT_RENAME[j])
            actuator.append(p)

    # Sensors: EE force/torque at ft_frame (robosuite robot.control reads these),
    # plus per-finger pos/vel/torque on the two actuated joints (MagpieHandBridge
    # reads these to report aperture/force/velocity). After robosuite merge they
    # are prefixed gripper0_<side>_<name>.
    sensor = ET.SubElement(mj, "sensor")
    ET.SubElement(sensor, "force", {"name": "force_ee", "site": "ft_frame"})
    ET.SubElement(sensor, "torque", {"name": "torque_ee", "site": "ft_frame"})
    for nm, jnt in (("left", "finger_joint1"), ("right", "finger_joint2")):
        ET.SubElement(sensor, "jointpos", {"name": f"{nm}_finger_pos", "joint": jnt})
        ET.SubElement(sensor, "jointvel", {"name": f"{nm}_finger_vel", "joint": jnt})
        ET.SubElement(sensor, "jointactuatorfrc", {"name": f"{nm}_finger_torque", "joint": jnt})

    return ET.ElementTree(mj)


def write_tree(tree, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    remap_geom_groups(tree.getroot())
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=False)
    with open(path, "r") as f:
        body = f.read()
    with open(path, "w") as f:
        f.write("<!-- AUTOGENERATED by CL_Assets/robosuite_assets/build_assets.py - do not edit by hand -->\n")
        f.write(body)


def main():
    write_tree(build_robot(), OUT_ROBOT)
    write_tree(build_gripper("magpie_right_gripper", GRIPPER_FRAG_R), OUT_GRIP_R)
    write_tree(build_gripper("magpie_left_gripper", GRIPPER_FRAG_L, strip_prefix="leftg_"), OUT_GRIP_L)
    print("wrote:\n ", OUT_ROBOT, "\n ", OUT_GRIP_R, "\n ", OUT_GRIP_L)

    # smoke test: compile each standalone with mujoco
    import mujoco
    for path in (OUT_ROBOT, OUT_GRIP_R, OUT_GRIP_L):
        m = mujoco.MjModel.from_xml_path(path)
        print(f"OK  {os.path.basename(path)}: nq={m.nq} nu={m.nu} nbody={m.nbody} "
              f"nsensor={m.nsensor}")

    # Guard: robot.xml MUST carry a free joint on the pelvis. robosuite only
    # preserves it under a NullBase (NullBaseModel); a MobileBaseModel base
    # (e.g. NoActuationBase) silently deletes it in add_mobile_base.
    mr = mujoco.MjModel.from_xml_path(OUT_ROBOT)
    has_free = any(mr.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(mr.njnt))
    assert has_free, "robot.xml lost its <freejoint> — the pelvis must keep a free joint"
    print("OK  robot.xml has a pelvis free joint")


if __name__ == "__main__":
    main()
