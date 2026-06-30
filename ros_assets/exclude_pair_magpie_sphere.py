import re
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent

urdf_file = SCRIPT_DIR / 'h1_2_magpie_sphere.urdf'
output_file = SCRIPT_DIR / 'h1_2_magpie_sphere_collision.srdf'


def read_urdf_links(filename):
    text = filename.read_text()
    return re.findall(r'<link\s+name="([^"]+)"', text)


def link_group(link, all_links):
    prefix = f'{link}_sphere'
    spheres = [name for name in all_links if name.startswith(prefix)]
    return [link] + sorted(spheres, key=lambda name: int(name.removeprefix(prefix)))


def write_disable_pair(file, link1, link2):
    if link1 == link2:
        return
    file.write(f'  <disable_collisions link1="{link1}" link2="{link2}" reason="Never"/>\n')


def write_group_self_collisions(file, group):
    for i, link1 in enumerate(group):
        for link2 in group[i + 1:]:
            write_disable_pair(file, link1, link2)


def write_group_pair_collisions(file, group1, group2):
    for link1 in group1:
        for link2 in group2:
            write_disable_pair(file, link1, link2)


all_links = read_urdf_links(urdf_file)

# Arm links stay unchanged. The Magpie gripper is represented for collision by
# the wrist_yaw_link spheres, so the enabled arm chain does not include grasp or
# finger links.
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

enabled_base_links = ['torso_link'] + left_arm_links + right_arm_links
enabled_links = set()
for link in enabled_base_links:
    enabled_links.update(link_group(link, all_links))

# Everything not in the torso/arm collision model is disabled, including lower
# body links, sensors, grasp frames, and any Magpie-specific links present in the
# URDF.
disabled_links = [link for link in all_links if link not in enabled_links]

output_file.parent.mkdir(parents=True, exist_ok=True)
with open(output_file, 'w') as f:
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    f.write('<!-- SRDF fragment: disable collisions for lower body, sensors, and Magpie grasp frames -->\n')
    f.write('<robot name="h1_2_sphere">\n\n')
    f.write('  <!-- disable_collisions entries -->\n')

    # Disable collisions between disabled links and all other links.
    for disabled_link in disabled_links:
        for other_link in all_links:
            write_disable_pair(f, disabled_link, other_link)

    # Disable self-collisions inside each enabled collision group: a link and
    # all sphere links that actually exist for it in the Magpie sphere URDF.
    write_group_self_collisions(f, link_group('torso_link', all_links))
    for link in left_arm_links:
        write_group_self_collisions(f, link_group(link, all_links))
    for link in right_arm_links:
        write_group_self_collisions(f, link_group(link, all_links))

    # Disable collisions between consecutive enabled links.
    write_group_pair_collisions(f, link_group('torso_link', all_links), link_group(left_arm_links[0], all_links))
    write_group_pair_collisions(f, link_group('torso_link', all_links), link_group(right_arm_links[0], all_links))
    for arm_links in [left_arm_links, right_arm_links]:
        for i in range(len(arm_links) - 1):
            write_group_pair_collisions(f, link_group(arm_links[i], all_links), link_group(arm_links[i + 1], all_links))

    # Disable collisions between second consecutive enabled links.
    for arm_links in [left_arm_links, right_arm_links]:
        for i in range(len(arm_links) - 2):
            write_group_pair_collisions(f, link_group(arm_links[i], all_links), link_group(arm_links[i + 2], all_links))

    f.write('\n</robot>\n')

print(f'SRDF file generated: {output_file}')
