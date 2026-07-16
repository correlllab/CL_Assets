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


def add_chain_pairs(pairs, chain, skip_distance=2):
    for index, link1 in enumerate(chain):
        for link2 in chain[index + 1:index + 1 + skip_distance]:
            add_pair(pairs, link1, link2)


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
left_gripper_chains = [
    [
        'left_wrist_yaw_link',
        'lg_mount',
        'lg_base_bot',
        'lg_base_top',
        'lg_left_crank',
        'lg_left_finger',
        'lg_left_rocker',
    ],
    [
        'lg_base_top',
        'lg_right_crank',
        'lg_right_finger',
        'lg_right_rocker',
    ],
]
right_gripper_chains = [
    [
        'right_wrist_yaw_link',
        'rg_mount',
        'rg_base_bot',
        'rg_base_top',
        'rg_left_crank',
        'rg_left_finger',
        'rg_left_rocker',
    ],
    [
        'rg_base_top',
        'rg_right_crank',
        'rg_right_finger',
        'rg_right_rocker',
    ],
]
torso_and_arm_links = ['torso_link'] + left_arm_links + right_arm_links
gripper_links = [
    link
    for chain in left_gripper_chains + right_gripper_chains
    for link in chain
]
enabled_links = set(torso_and_arm_links + gripper_links)
disabled_links = [link for link in all_links if link not in enabled_links]

disabled_pairs = set()
for disabled_link in disabled_links:
    for other_link in all_links:
        add_pair(disabled_pairs, disabled_link, other_link)

add_chain_pairs(disabled_pairs, ['torso_link'] + left_arm_links)
add_chain_pairs(disabled_pairs, ['torso_link'] + right_arm_links)
for chain in left_gripper_chains + right_gripper_chains:
    add_chain_pairs(disabled_pairs, chain)

OUTPUT_FILE.write_text(
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<!-- SRDF fragment: disable collisions for non-robot and adjacent links -->\n'
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
