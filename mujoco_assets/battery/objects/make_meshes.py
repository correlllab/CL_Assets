"""Build the visual meshes for the part files in this directory.

Run from anywhere: python make_meshes.py (needs numpy + trimesh). Writes
assets/fuse_block.stl in metres, centred on the fuse block body origin.
"""
import os

import numpy as np
from trimesh.creation import box
from trimesh.transformations import euler_matrix, translation_matrix
from trimesh.util import concatenate

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def make_part(half_extents, pos=(0, 0, 0), euler_deg=(0, 0, 0)):
    """Box from MuJoCo-style half-extents (trimesh takes full extents)."""
    mesh = box(extents=[e * 2 for e in half_extents])
    rot = euler_matrix(*np.radians(euler_deg), "sxyz")
    mesh.apply_transform(translation_matrix(pos) @ rot)
    return mesh


def fuse_block():
    return concatenate([
        make_part([0.035, 0.011, 0.010], pos=[0, 0.002, 0.002]),   # bridge
        make_part([0.012, 0.013, 0.012], pos=[-0.023, 0, 0]),      # left lobe
        make_part([0.012, 0.013, 0.012], pos=[0.023, 0, 0]),       # right lobe
        make_part([0.003, 0.004, 0.002], pos=[0.037, 0, 0.006]),   # end tab
    ])


if __name__ == "__main__":
    os.makedirs(ASSETS, exist_ok=True)
    out = os.path.join(ASSETS, "fuse_block.stl")
    fuse_block().export(out)
    print(f"saved {out}")
