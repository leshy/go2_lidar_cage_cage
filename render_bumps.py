# /// script
# dependencies = ["numpy", "matplotlib"]
# ///
"""3D renders of the bump FEA: deformed shape (exaggerated) coloured by von Mises, undeformed ghost, load arrow.

    uv run render_bumps.py      # reads $FEA_OUT/fields.npz -> $FEA_OUT/render.png
"""
import json, os
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

S_Z = 25.0                                   # PETG across-layer strength, colour scale max
out = Path(__file__).parent / os.environ.get("FEA_OUT", "outputs/fea")
z = np.load(out / "fields.npz")
p, bf, bf_el = z["p"], z["bf"], z["bf_el"]
cases = json.loads(str(z["cases"]))
summ = json.loads(str(z["summary"])) if "summary" in z.files else {}
names = list(cases)
fig = plt.figure(figsize=(5.2 * len(names), 10.5))
for j, name in enumerate(names):
    u, s = z[f"{name}__u"], z[f"{name}__s"]
    dmax = np.linalg.norm(u, axis=1).max()
    k = 6.0 / dmax if dmax > 0 else 1.0       # exaggerate so the worst point moves ~6 mm on screen
    q = p + k * u
    col = plt.cm.inferno(np.clip(s[bf_el] / S_Z, 0, 1))
    pt, Fv = np.array(cases[name][0]), np.array(cases[name][1])
    for i, (el, az) in enumerate(((28, -125), (12, -35))):
        ax = fig.add_subplot(2, len(names), i * len(names) + j + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(p[bf], facecolor=(0.6, 0.6, 0.6, 0.06), edgecolor="none"))
        ax.add_collection3d(Poly3DCollection(q[bf], facecolors=col, edgecolor="none"))
        a = -Fv / np.linalg.norm(Fv) * 30
        ax.quiver(*(pt + a), *(-a), color="cyan", linewidth=3, arrow_length_ratio=0.3)
        ax.set_xlim(-65, 75); ax.set_ylim(-70, 70); ax.set_zlim(-20, 75); ax.set_box_aspect((140, 140, 95))
        ax.view_init(el, az); ax.set_axis_off()
        if i == 0:
            r = summ.get(name, {})
            rx = r.get("reactions_N", {})
            ax.set_title(f"{name}  hit {np.linalg.norm(Fv):.0f} N\n"
                         f"p99.9 {np.percentile(s, 99.9):.0f} MPa, at hit {r.get('defl_mm', 0):.1f} mm (shown x{k:.1f})\n"
                         f"cracks ~{r.get('crack_N', 0)} N, touches cap ~{r.get('touch_N')} N\n"
                         + "  ".join(f"{a} {v} N" for a, v in rx.items()), fontsize=9)
sm = plt.cm.ScalarMappable(cmap="inferno", norm=plt.Normalize(0, S_Z))
fig.colorbar(sm, ax=fig.axes, shrink=0.5, label=f"von Mises MPa (white/yellow >= {S_Z} = PETG layer strength) @ 300 N")
fig.savefig(out / "render.png", dpi=75, bbox_inches="tight")
print(out / "render.png")
