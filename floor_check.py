# /// script
# dependencies = ["trimesh", "numpy", "scipy", "pillow", "matplotlib"]
# ///
"""Side view of the cage on the robot lying down, and its clearance to the floor.

    uv run floor_check.py [outputs/assembly-build_print/assembly.stl]   # -> docs/floor_side.png

Needs mujoco_menagerie's unitree_go2 base_0..4.obj in ref/ (see README). The robot rests on its belly
charging contacts and rear belly, nose up ~5.4 deg; the floor is the line through those.
"""
import sys
from pathlib import Path

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

HERE = Path(__file__).parent
PITCH = 2.8782                                   # radar_joint pitch (URDF)
ORIGIN = np.array([0.28945, 0, -0.046825])       # radar origin in the base frame, m
NOSE_UP = np.radians(5.4)

R = np.array([[np.cos(PITCH), 0, np.sin(PITCH)], [0, 1, 0], [-np.sin(PITCH), 0, np.cos(PITCH)]])
load = lambda f: trimesh.load(f, force="mesh")
body = [load(HERE / f"ref/base_{i}.obj") for i in (2, 3, 4)]
cap = [load(HERE / f"ref/base_{i}.obj") for i in (0, 1)]    # spinning stock cage
cage = load(sys.argv[1] if len(sys.argv) > 1 else HERE / "outputs/assembly-build_print/assembly.stl")
cage.vertices = cage.vertices / 1000 @ R.T + ORIGIN          # lidar frame mm -> base frame m

v = np.vstack([b.vertices for b in body])
contacts = v[v[:, 2] < v[:, 2].min() + 0.002]
p0 = contacts[contacts[:, 2].argmin()]
up = np.array([np.sin(NOSE_UP), 0, np.cos(NOSE_UP)])        # floor normal in the base frame
print(f"cage above floor: {((cage.vertices - p0) @ up).min() * 1000:.1f} mm")

fig, ax = plt.subplots(figsize=(10, 5))
for meshes, col in ((body, "#bbbbbb"), (cap, "#333333"), ([cage], "#e07020")):
    for m in meshes:
        ax.add_collection(PolyCollection(m.vertices[m.faces][:, :, [0, 2]] * 1000, facecolor=col, edgecolor="none", alpha=0.85))
x = np.array([0.05, 0.4])
ax.plot(x * 1000, (p0[2] - (x - p0[0]) * np.tan(NOSE_UP)) * 1000, "b-", lw=1, label="floor (robot lying down)")
ax.plot(contacts[:, 0] * 1000, contacts[:, 2] * 1000, "g.", ms=3, label="belly charging contacts")
ax.set_xlim(50, 400); ax.set_ylim(-150, 0); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("robot x, mm (front ->)"); ax.set_ylabel("robot z, mm"); ax.legend(loc="upper left", fontsize=8)
ax.set_title("Go2 side view: body (grey), stock spinning cage (dark), outer cage (orange)")
(HERE / "docs").mkdir(exist_ok=True)
fig.tight_layout(); fig.savefig(HERE / "docs/floor_side.png", dpi=110)
