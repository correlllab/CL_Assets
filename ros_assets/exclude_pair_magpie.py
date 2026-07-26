import re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
URDF_FILE = SCRIPT_DIR / 'h1_2_magpie.urdf'
OUTPUT_FILE = SCRIPT_DIR / 'h1_2_magpie_collision.srdf'


def read_urdf_links(filename):
    '''Read link names from a URDF file'''
    return re.findall(r'<link\s+name="([^"]+)"', filename.read_text())


def pair_key(link1, link2):
    return tuple(sorted((link1, link2)))


def add_pair(pairs, link1, link2):
    if link1 != link2:
        pairs.add(pair_key(link1, link2))


def add_group_self_collisions(pairs, group):
    for index, link1 in enumerate(group):
        for link2 in group[index + 1:]:
            add_pair(pairs, link1, link2)


def add_group_pair_collisions(pairs, group1, group2):
    for link1 in group1:
        for link2 in group2:
            add_pair(pairs, link1, link2)


def add_chain_exclusions(pairs, chain, distance):
    for index, group in enumerate(chain):
        if index + distance < len(chain):
            add_group_pair_collisions(pairs, group, chain[index + distance])


all_links = read_urdf_links(URDF_FILE)

left_arm_links = [
    'left_shoulder_pitch_link',
    'left_shoulder_roll_link',
    'left_shoulder_yaw_link',
    'left_elbow_link',
    'left_wrist_roll_link',
    'left_wrist_pitch_link',
    'left_wrist_yaw_link',
]
right_arm_links = [
    'right_shoulder_pitch_link',
    'right_shoulder_roll_link',
    'right_shoulder_yaw_link',
    'right_elbow_link',
    'right_wrist_roll_link',
    'right_wrist_pitch_link',
    'right_wrist_yaw_link',
]
left_gripper_links = [
    'lg_mount',
    'lg_base_bot',
    'lg_base_top',
    'lg_left_crank',
    'lg_left_finger',
    'lg_left_rocker',
    'lg_right_crank',
    'lg_right_finger',
    'lg_right_rocker',
]
right_gripper_links = [
    'rg_mount',
    'rg_base_bot',
    'rg_base_top',
    'rg_left_crank',
    'rg_left_finger',
    'rg_left_rocker',
    'rg_right_crank',
    'rg_right_finger',
    'rg_right_rocker',
]


def arm_collision_groups(arm_links, gripper_links):
    return [[link] for link in arm_links[:-1]] + [
        [arm_links[-1], *gripper_links],
    ]


left_groups = arm_collision_groups(left_arm_links, left_gripper_links)
right_groups = arm_collision_groups(right_arm_links, right_gripper_links)
enabled_links = {
    'torso_link',
    *[
        link
        for group in left_groups + right_groups
        for link in group
    ],
}
disabled_links = [link for link in all_links if link not in enabled_links]

disabled_pairs = set()
for disabled_link in disabled_links:
    for other_link in all_links:
        add_pair(disabled_pairs, disabled_link, other_link)

torso_group = ['torso_link']
for group in [torso_group, *left_groups, *right_groups]:
    add_group_self_collisions(disabled_pairs, group)

for arm_groups in [left_groups, right_groups]:
    add_group_pair_collisions(disabled_pairs, torso_group, arm_groups[0])
    add_chain_exclusions(disabled_pairs, arm_groups, distance=1)
    add_chain_exclusions(disabled_pairs, arm_groups, distance=2)

OUTPUT_FILE.write_text(
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<!-- SRDF fragment: disable non-robot, group-self, and near-chain collisions -->\n'
    '<robot name="h1_2_magpie">\n\n'
    '  <!-- disable_collisions entries -->\n'
    + ''.join(
        f'  <disable_collisions link1="{link1}" link2="{link2}" '
        'reason="Never"/>\n'
        for link1, link2 in sorted(disabled_pairs)
    )
    + '\n</robot>\n',
)

print(f'Generated {len(disabled_pairs)} collision exclusions in {OUTPUT_FILE}')
