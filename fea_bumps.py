# /// script
# dependencies = ["gmsh", "scikit-fem", "pypardiso", "matplotlib", "numpy"]
# ///
"""Linear-static bump FEA of the lidar cage: gmsh P2 tets, scikit-fem, pardiso.

    MKL_NUM_THREADS=4 uv run fea_bumps.py [outputs/assembly-build_print/assembly.step] [h]
    # -> $FEA_OUT (default outputs/fea): bumps.json, bumps.png, fields.npz (for render_bumps.py)

Frame = assembly.py's lidar frame (mm, N, MPa). Each case pushes F on a ~7 mm patch of the outer
surface, at the outermost point in the hit direction. Three holds (the truth is between clamp and grip):
  bolts : the two M4 hole walls fixed, nothing else (worst case)
  clamp : + ring bore on stiff springs along the bore normal (frictionless contact) + tab underside
          on stiff normal springs (resting on the bracket). Bilateral: pull-off is not modelled.
  grip  : as clamp, but the bore held in every direction: ideal clamp friction (upper bound)
Material: PETG FDM. Strength in-plane ~45 MPa, across layers ~25 MPa (the hoops print vertical, so
leg bending loads the layer bonds).
"""
import importlib.util, json, math, os, sys
from pathlib import Path

if "PYPARDISO_MKL_RT" not in os.environ:
    lib = Path(importlib.util.find_spec("pypardiso").origin).parents[3]
    os.environ["PYPARDISO_MKL_RT"] = str(next(lib.glob("libmkl_rt.so*"), ""))

import gmsh
import numpy as np
import pypardiso
from skfem import Basis, ElementTetP2, ElementVector, FacetBasis, LinearForm, BilinearForm, MeshTet, asm, condense
from skfem.helpers import dot, sym_grad
from skfem.models.elasticity import lame_parameters, linear_elasticity

HERE = Path(__file__).parent
E, NU = 2000.0, 0.38                     # PETG
S_XY, S_Z = 45.0, 25.0                   # in-plane / across-layer strength
F = 300.0                                # N per case (~2x robot weight, static equivalent)
CLEAR_TO_CAP = 4.0                       # mm the hoops can bend before touching the spinning cap (assembly.CLEAR;
                                         # 3 at the crown, 1.6 at the thin back corner)

# geometry from assembly.py (keep in sync): screws on the bracket plane through (42,0,-1.5), slope 20.4 deg
SCREW_X, SCREW_PITCH, BRACKET_DEG, M4_R = 56.5, 20.0, 20.4, 2.25
RING_TOP = -2.0

s45 = math.sqrt(0.5)
# case: hit direction (the cage surface faces it), and the band the impact point is looked for in.
# The impact point is the outermost surface point along that direction, so any hoop shape compares fairly.
CASES = {
    "front_mid":      ((-1, 0, 0), lambda c: (abs(c[:, 1]) < 5) & (abs(c[:, 2] - 25) < 5)),
    "front_shoulder": ((-s45, 0, s45), lambda c: abs(c[:, 1]) < 5),
    "apex_down":      ((0, 0, 1), lambda c: (abs(c[:, 1]) < 5) & (abs(c[:, 0]) < 5)),
    "side_mid":       ((0, 1, 0), lambda c: (abs(c[:, 0]) < 5) & (abs(c[:, 2] - 25) < 5)),
    "side_shoulder":  ((0, s45, s45), lambda c: abs(c[:, 0]) < 5),
}
# grey housing outer radius vs z (assembly.HOUSING_PROFILE) + HUG_CLEAR: the ring bore
BORE_Z = [-30, -8, 0]              # measured 87 mm housing + 0.4 for 8 mm, then the 45 deg clearance flare
BORE_R = [43.9 + 22, 43.9, 43.9]


def mesh(step, h):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.importShapes(str(step))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", h)
    gmsh.option.setNumber("Mesh.MeshSizeMin", h / 3)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.model.mesh.generate(3)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    p = np.zeros((tags.max() + 1, 3)); p[tags] = xyz.reshape(-1, 3)
    _, et = gmsh.model.mesh.getElementsByType(4)
    t = et.reshape(-1, 4)
    gmsh.finalize()
    used = np.unique(t); remap = -np.ones(len(p), int); remap[used] = np.arange(len(used))
    return MeshTet(p[used].T, remap[t].T)


def facet_geom(m):
    f = m.boundary_facets()
    tri = m.p[:, m.facets[:, f]]                          # 3 x 3 x nf
    c = tri.mean(axis=1).T
    n = np.cross((tri[:, 1] - tri[:, 0]).T, (tri[:, 2] - tri[:, 0]).T)
    area = 0.5 * np.linalg.norm(n, axis=1)
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    # orient outward: away from the owning element's centroid
    el = m.f2t[0, f]
    ec = m.p[:, m.t[:, el]].mean(axis=1).T
    n *= np.sign(np.einsum("ij,ij->i", n, c - ec))[:, None]
    return f, c, n, area


def main():
    step = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "outputs/assembly-build_print/assembly.step"
    h = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    m = mesh(step, h)
    e = ElementVector(ElementTetP2())
    ib = Basis(m, e)
    print(f"mesh: {m.t.shape[1]} tets, {ib.N} dofs", flush=True)
    K = asm(linear_elasticity(*lame_parameters(E, NU)), ib)

    f, c, n, area = facet_geom(m)
    a = math.radians(BRACKET_DEG)
    nb = np.array([math.sin(a), 0, math.cos(a)])          # bracket normal
    holes = []
    for sy in (-1, 1):
        x0 = np.array([SCREW_X, sy * SCREW_PITCH / 2, -1.5 - math.tan(a) * (SCREW_X - 42)])
        ax = np.array([0.0, 0, 1])                                  # M4s run parallel to the lidar axis
        d = c - x0; radial = d - np.outer(d @ ax, ax)
        holes.append(np.abs(np.linalg.norm(radial, axis=1) - M4_R) < 0.3)
    fixed = f[np.any(holes, axis=0)]
    print(f"M4 wall facets: {len(fixed)}", flush=True)
    D = ib.get_dofs(facets=fixed).all()

    # clamp springs: ring bore (radial) + tab underside on the bracket (normal)
    r_xy = np.hypot(c[:, 0], c[:, 1])
    # bore facets: on the bore surface r = R(z) and facing along its normal (-r_hat, R'(z)) -- the ring's end
    # faces at the bracket gap also sit at bore radius, and springs on them would pin the ring ends sideways
    Rz = np.interp(c[:, 2], BORE_Z, BORE_R)
    dR = (np.interp(c[:, 2] + 0.01, BORE_Z, BORE_R) - np.interp(c[:, 2] - 0.01, BORE_Z, BORE_R)) / 0.02
    rh = np.stack([c[:, 0], c[:, 1], 0 * c[:, 0]], axis=1) / r_xy[:, None]
    n_exp = np.concatenate([-rh[:, :2], dR[:, None]], axis=1) / np.sqrt(1 + dR ** 2)[:, None]
    bore = (np.abs(r_xy - Rz) < 0.4) & (c[:, 2] < RING_TOP + 0.1) & (c[:, 2] > -8.1) & (np.einsum("ij,ij->i", n, n_exp) > 0.95)  # measured band only: the flare below is clearance
    plane_d = (c - np.array([42, 0, -1.5])) @ nb
    under = (np.abs(plane_d - 0.5) < 0.3) & (n @ nb < -0.9)      # tab gap 0.5 above the bracket
    k_spring = 200.0                                              # N/mm^3, stiff foundation
    # Frictionless contact: stiff springs along the TRUE surface normal only. The mesh's facet normals are a
    # little off; with springs this stiff that leaks tangential/axial grip (A/B: half the stress), so the
    # normal comes from the geometry: the bore profile (-r_hat, R'(z)) and the bracket plane.
    def bore_normal(x):
        r = np.hypot(x[0], x[1])
        dR = (np.interp(x[2] + 0.01, BORE_Z, BORE_R) - np.interp(x[2] - 0.01, BORE_Z, BORE_R)) / 0.02
        v = np.stack([-x[0] / r, -x[1] / r, dR])
        return v / np.linalg.norm(v, axis=0)

    def springs(sel, normal_of):
        fb = FacetBasis(m, e, facets=f[sel])
        @BilinearForm
        def kn(u, v, w):
            nn = normal_of(w.x)
            return k_spring * dot(u, nn) * dot(v, nn)
        return asm(kn, fb)
    K_bore = springs(bore, bore_normal)
    K_brk = springs(under, lambda x: np.broadcast_to(nb[:, None, None], x.shape))
    # grip: the clamp's friction holds the bore in every direction (upper bound; frictionless is the lower)
    K_grip = asm(BilinearForm(lambda u, v, w: k_spring * dot(u, v)), FacetBasis(m, e, facets=f[bore]))
    Kc = K + K_bore + K_brk
    Kg = K + K_grip + K_brk
    supports = {"bolts": [], "clamp": [("ring", K_bore), ("bracket", K_brk)], "grip": [("ring", K_grip), ("bracket", K_brk)]}
    comp = np.zeros(ib.N, int)                           # x/y/z component of every dof (vertex + edge dofs)
    for k in range(3):
        comp[ib.nodal_dofs[k]] = k
        if hasattr(ib, "edge_dofs") and ib.edge_dofs.size:
            comp[ib.edge_dofs[k]] = k
    def vec(r, idx=slice(None)):                          # sum a dof vector into an (Fx, Fy, Fz) force
        return np.array([r[idx][comp[idx] == k].sum() for k in range(3)])
    print(f"bore facets {bore.sum()}, bracket facets {under.sum()}", flush=True)

    # element stress helpers
    lam, mu = lame_parameters(E, NU)
    def vm(u):
        du = sym_grad(ib.interpolate(u))
        tr = du[0, 0] + du[1, 1] + du[2, 2]
        s = 2 * mu * du + lam * np.eye(3)[:, :, None, None] * tr
        d = s - np.eye(3)[:, :, None, None] * (s[0, 0] + s[1, 1] + s[2, 2]) / 3
        return np.sqrt(1.5 * np.einsum("ij...,ij...->...", d, d))   # (nelem, nqp)

    ectr = m.p[:, m.t].mean(axis=1).T
    results, maps, fields = {}, {}, {}
    impact = {}
    for name, (dirn, band) in CASES.items():
        dirn = np.array(dirn, float); fh = -dirn; Fv = F * fh
        cand = np.flatnonzero(band(c) & (n @ dirn > 0.3))
        pt = c[cand[np.argmax(c[cand] @ dirn)]]
        impact[name] = (pt.tolist(), Fv.tolist())
        sel = (np.linalg.norm(c - pt, axis=1) < 7) & (n @ dirn > 0.3)
        A = area[sel].sum()
        fb = FacetBasis(m, e, facets=f[sel])
        tr = Fv / A
        b = asm(LinearForm(lambda v, w: dot(np.broadcast_to(tr[:, None, None], v.shape), v)), fb)
        for hold, KK in (("bolts", K), ("clamp", Kc), ("grip", Kg)):
            u = np.zeros(ib.N)
            u[ib.complement_dofs(D)] = pypardiso.spsolve(*condense(KK, b, D=D, expand=False))
            # where the 300 N goes (N): spring supports carry -K_s u, the bolts the rest (reaction at D)
            react = {nm: -vec(Ks @ u) for nm, Ks in supports[hold]}
            react["M4 bolts"] = vec(KK @ u - b, D) - sum((vec(Ks @ u, D) for _, Ks in supports[hold]), np.zeros(3))
            s = vm(u).max(axis=1)
            idx = np.argsort(s)[::-1]
            hot = []                                            # distinct hot spots, 12 mm apart
            for i in idx[: max(20, len(idx) // 200)]:
                if all(np.linalg.norm(ectr[i] - ectr[j]) > 12 for j in hot):
                    hot.append(i)
                if len(hot) == 4:
                    break
            # deflection at the load patch along F
            nodes_u = u[ib.nodal_dofs].T                        # vertex displacements
            vid = np.unique(m.facets[:, f[sel]])
            defl = float((nodes_u[vid] @ fh).mean())
            s999 = float(np.percentile(s, 99.9))
            gap = CLEAR_TO_CAP
            results[f"{name}/{hold}"] = dict(
                crack_N=round(F * S_Z / s999), touch_N=round(F * gap / abs(defl)) if abs(defl) > 1e-6 else None,
                reactions_N={k: round(float(np.linalg.norm(v))) for k, v in react.items()},
                F=Fv.tolist(), at=np.round(pt, 1).tolist(), patch_mm2=round(A, 1), max_MPa=round(float(s.max()), 1), p999_MPa=round(s999, 1),
                defl_mm=round(defl, 2), SF_layers=round(S_Z / s999, 2), SF_inplane=round(S_XY / s999, 2),
                hotspots=[dict(xyz=np.round(ectr[i], 1).tolist(), MPa=round(float(s[i]), 1)) for i in hot])
            maps[f"{name}/{hold}"] = s
            fields[f"{name}/{hold}"] = (nodes_u.astype(np.float32), s.astype(np.float32))
            r = results[f"{name}/{hold}"]
            print(f"{name:15s} {hold:5s} p99.9 {r['p999_MPa']:6.1f} MPa  defl {r['defl_mm']:5.2f} mm  "
                  f"cracks@{r['crack_N']:5d} N  touches cap@{r['touch_N']} N  carried by {r['reactions_N']}", flush=True)

    out = HERE / os.environ.get("FEA_OUT", "outputs/fea"); out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "fields.npz", p=m.p.T, t=m.t.T, bf=m.facets[:, f].T, bf_el=m.f2t[0, f],
                        **{k.replace("/", "__") + "__u": v[0] for k, v in fields.items()},
                        **{k.replace("/", "__") + "__s": v[1] for k, v in fields.items()},
                        cases=json.dumps(impact), summary=json.dumps(results))
    (out / "bumps.json").write_text(json.dumps(dict(h=h, E=E, F=F, results=results), indent=1))
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    names = list(CASES)
    fig, axs = plt.subplots(2, len(names), figsize=(4.2 * len(names), 8))
    for j, name in enumerate(names):
        s = maps[f"{name}/clamp"]; o = np.argsort(s)
        for i, (ix, iy, lab) in enumerate(((0, 1, "top (x,y)"), (0, 2, "side (x,z)"))):
            sc = axs[i, j].scatter(ectr[o, ix], ectr[o, iy], c=s[o], s=1.5, cmap="inferno", vmin=0, vmax=S_Z)
            axs[i, j].set_aspect("equal"); axs[i, j].set_title(f"{name} [clamp] {lab}", fontsize=9)
            pt, Fv = impact[name]
            axs[i, j].annotate("", xy=(pt[ix], pt[iy]), xytext=(pt[ix] - Fv[ix] / 12, pt[iy] - Fv[iy] / 12),
                               arrowprops=dict(color="cyan", width=2))
    fig.colorbar(sc, ax=axs, shrink=0.6, label=f"von Mises MPa (scale max = {S_Z} = layer strength)")
    fig.savefig(out / "bumps.png", dpi=80)
    print(f"-> {out/'bumps.json'}, {out/'bumps.png'}")


if __name__ == "__main__":
    main()
