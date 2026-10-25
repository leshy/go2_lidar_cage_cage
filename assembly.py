"""Unitree Go2 L1 lidar outer cage: a fixed cage around the stock black cage.

    khana check  assembly.py
    khana view   assembly.py:build_view    # viewer: http://localhost:3939/viewer
    khana export assembly.py:build_print

The stock black cage is bolted to the lidar's SPINNING head, so when it hits something the
lidar stalls. This cage is fixed to the robot instead. One printed piece:
- ring hugs the grey (non-spinning) head housing below the rim (everything above the rim
  spins); two M3 pinch clamps squeeze it onto the housing
- tab bolts onto the grey chin bracket with its 2x M4 screws (20 mm apart, bracket 35 mm wide);
  reuse the holes with M4 screws ~4 mm longer than stock
- two 10x10 mm hoops (fore-aft + lateral) arch over the spinning cage

Coordinate frame (lidar frame, mm):
    origin = URDF radar origin (base +0.28945, 0, -0.046825 m)
    +Z     = lidar axis out of the head (on the robot: down and 15° forward, pitch 2.8782 rad)
    -X     = robot forward, +X = toward the body along the chin bracket, +Y = robot left
Shapes come from mujoco_menagerie unitree_go2 (base_0/1/4), corrected with numbers measured on the
robot (marked MEASURED) and test prints (marked FITTED).
"""
import math

from build123d import (Align, Axis, Box, Color, Cylinder, Part, Plane, Polygon, Pos, Rectangle, RectangleRounded,
                       RegularPolygon, Rot, extrude, import_stl, revolve)

from cad_khana.mechanism.assembly import Assembly

# --- spinning stock cage + lidar head: (r, z) max-radius profile; starts at z≈-2 (the grey rim), below
# that it sits inside the grey housing. MEASURED: cap 84 mm diameter, 56 mm tall above the rim; the
# shape is the mesh's profile scaled to those numbers (mesh: ~86 x 52).
SPIN_R, SPIN_TOP = 84.0 / 2, -2 + 56.0
_MESH_SPIN = [(0, -2), (43, -2), (43, 28), (42.3, 28), (41.3, 32), (39.9, 36), (38.9, 40),
              (36.6, 42), (35.2, 44), (33.6, 46), (29.9, 48), (19.5, 50), (0, 50.2)]
SPIN_PROFILE = [(r * SPIN_R / 43, -2 + (z + 2) * (SPIN_TOP + 2) / 52.2) for r, z in _MESH_SPIN]
# grey housing: MEASURED 87 mm diameter for ~8 mm below the rim (the mesh says 90.4). Below that the
# bore flares out at 45 deg: clearance only, it doesn't seat there.
HOUSING_R = 87.0 / 2
HOUSING_PROFILE = [(0, -30), (HOUSING_R + 22, -30), (HOUSING_R, -8), (HOUSING_R, 0), (0, 0)]
# chin bracket top: plane through (42, 0, -1.5), sloping down toward +X
BRACKET_DEG = 20.4          # from the mesh; FITTED (the tab sits flat on it)
BRACKET_W = 35.0            # MEASURED (mesh says 31)
SCREW_X = 56.5              # M4 centres from the lidar axis along X; FITTED (2nd print)
SCREW_PITCH = 20.0          # MEASURED
TAB_END = 69.5              # tab end from the lidar axis (the bracket flattens past x≈70)

# --- cage
CLEAR = 4.0                 # to the spinning cage
RIB = 10.0                  # hoop width sideways (what the lidar sees)
RIB_D = 10.0                # hoop depth in the arch's plane (radial on the legs): adds no lidar shadow
RING_TOP, RING_BOT = -2.0, -10.0  # top = where spinning starts; bottom 2 mm into the 45 deg clearance flare
RING_W = 6.0                # radial wall
ROOT_FILLET = 4.0           # leg-to-ring-top fillet wedges along both leg sides
FILLET_DEG = 18.0           # plan-view leg-to-ring triangles reach this far along the ring
HUG_CLEAR = 0.4             # ring bore to housing
TAB_T, TAB_WING, TAB_GAP = 7.0, 6.0, 0.5
# lying down, the robot rests on its 4 belly charging contacts, nose up ~5.4 deg (menagerie belly mesh,
# FITTED): the floor is then 20.5 deg off the lidar axis toward +X, FLOOR_S mm out along that direction,
# only ~7 mm past the spinning cage's back shoulder. Everything is trimmed FLOOR_MARGIN short of the
# floor so the contacts still touch it; the fore-aft hoop's back corner gets a flat inner face
# BACK_CLEAR off the cap, which leaves it ~4.5 mm deep there (it mostly ties the tab to the cage).
FLOOR_DEG, FLOOR_S, FLOOR_MARGIN = 20.5, 65.0, 0.3
BACK_CLEAR = 1.6
CROWN_CLEAR = 3.0           # over the cap top: keeps the side hoop's crown 10 deep under the floor trim
INNER_SHOULDER_R = 28.0     # largest that keeps CROWN_CLEAR off the cap's shoulder (pulls the back corner in)

M4_CLEAR_D, M4_HEAD_D = 4.5, 9.5
M4_HEAD_INDENT = 1.0        # head pocket depth below the tab top
# pinch clamps: a slit through the ring just past each side leg's back triangle; an M3 across each
# slit closes it, the hoops flex and the ring squeezes the housing
SLIT_W = 4.0                # ~0.9 mm radial squeeze beyond HUG_CLEAR before the slits bottom out
EAR_L, EAR_H, BOSS_W = 9.0, 8.0, 8.0    # rear ear length past the slit, boss height, boss width (hole centred: 4 mm each way)
REAR_GUSSET_H, REAR_GUSSET_DEG = 15.0, 60.0  # rear leg-to-tab gusset: height above the tab top, slope
M3_CLEAR_D, M3_HEAD_D, M3_POCKET, M3_LEN = 3.2, 6.2, 3.0, 25.0  # pocket = head height; hole runs through, longer is fine
M3_NUT_AF, M3_NUT_CLEAR, M3_NUT_T = 5.5, 0.3, 2.9  # nut across flats, print clearance, slot thickness
NUT_WALL = 8.0              # rear ear material between slit and nut (the nut pulls against it)
SLIT_FWD = 5.0              # slit moved this far from the triangle tip toward the screw head: more ear behind the nut


def revolved(profile: list[tuple[float, float]], dr: float = 0.0) -> Part:
    pts = [(r + dr if r > 0 else 0, z) for r, z in profile]
    return revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z)


def ccw(pts) -> Polygon:
    """Polygon wound CCW in its own plane: a CW one has its face reversed and extrudes the wrong way."""
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return Polygon(*(pts if area > 0 else pts[::-1]), align=None)


def spin_envelope() -> Part:
    return revolved(SPIN_PROFILE)


def on_bracket(shape: Part, x: float, y: float = 0.0, deg: float = BRACKET_DEG) -> Part:
    """Place a shape built with local Z = bracket normal at bracket point (x, y)."""
    z = -1.5 - math.tan(math.radians(deg)) * (x - 42)
    return Pos(x, y, z) * Rot(0, deg, 0) * shape


def cage(clear: float = CLEAR, rib: float = RIB, screw_x: float = SCREW_X) -> Part:
    in_half_w = SPIN_R + clear + 1
    in_top = SPIN_TOP + CROWN_CLEAR
    def rr(w, top, r):  # tall rounded rect in the XZ plane with its top edge at `top`
        return Plane.XZ * Pos(0, top - 200) * RectangleRounded(w, 400, r)

    outer = rr(2 * (in_half_w + RIB_D), in_top + RIB_D, INNER_SHOULDER_R + RIB_D)
    inner = rr(2 * in_half_w, in_top, INNER_SHOULDER_R)
    # fore-aft hoop: the back corner's inner face is pulled in toward the cap, BACK_CLEAR off its
    # extreme toward the floor (the floor trim takes the outside)
    a = math.radians(FLOOR_DEG)
    s = max(math.sin(a) * r + math.cos(a) * z for r, z in SPIN_PROFILE) + BACK_CLEAR - 200
    near_side = Plane.XZ * Pos(math.sin(a) * s, math.cos(a) * s) * Rot(0, 0, -FLOOR_DEG) * Rectangle(400, 400)
    above_bot = Plane.XZ * Pos(0, RING_BOT + 100) * RectangleRounded(400, 200, 1)
    hoop_y = Rot(0, 0, 90) * extrude((outer - inner) & above_bot, amount=rib / 2, both=True)
    hoop_x = extrude((outer - (inner & near_side)) & above_bot, amount=rib / 2, both=True)

    # ring only locates the cage on the housing (the M4 tab holds it): slim, and a C that stops
    # at the bracket edges; the tab bridges the gap
    hug_bore = revolved(HOUSING_PROFILE, HUG_CLEAR)
    ring_ro = HOUSING_R + HUG_CLEAR + RING_W  # housing radius + wall
    gap_w = BRACKET_W + 2 * HUG_CLEAR
    # nothing inside the ring's outer radius over the bracket width, up to the ring top: no ring
    # there at all, the tab starts at the ring's outer arc
    # straight cut ending exactly at the ring's outer corner on the gap edge: an arc here met the
    # ring's straight end face in a double corner with a sliver between them
    x_cut = math.sqrt(ring_ro ** 2 - (gap_w / 2) ** 2)
    no_ring = Pos(x_cut / 2, 0, RING_TOP - 50) * Box(x_cut, gap_w, 100)
    ring = Pos(0, 0, (RING_BOT + RING_TOP) / 2) * Cylinder(ring_ro, RING_TOP - RING_BOT) - hug_bore - no_ring
    spin_bore = Pos(0, 0, RING_TOP - 0.01 + (rib + 12) / 2) * Cylinder(SPIN_R + clear, rib + 12)
    slot = on_bracket(Pos(40, 0, TAB_GAP - 20) * Box(80, gap_w, 40), 40)

    # tab planform: inner edge on the ring's outer arc, sides run tangent from the end corners
    # to the ring (the triangular flares). Top flush with the ring top, then follows the bracket
    # slope down; bottom sits on the bracket (slot cut), wings beside it reach the ring bottom.
    half_w = BRACKET_W / 2 + TAB_WING

    def tangent_pt(sign):
        d = math.hypot(TAB_END, half_w)
        th = math.atan2(sign * half_w, TAB_END) + sign * math.acos(ring_ro / d)
        return (ring_ro * math.cos(th), ring_ro * math.sin(th))

    planform = Polygon((0, 0), tangent_pt(-1), (TAB_END, -half_w), (TAB_END, half_w), tangent_pt(1), align=None)
    prism = Pos(0, 0, RING_BOT) * extrude(planform, amount=RING_TOP - RING_BOT)
    below_top = on_bracket(Pos(0, 0, TAB_GAP + TAB_T - 100) * Box(400, 400, 200), 42)
    tab = (prism - hug_bore - spin_bore) & below_top

    screws = []
    # M4s run parallel to the lidar axis (not square to the sloped bracket face)
    z_b = -1.5 - math.tan(math.radians(BRACKET_DEG)) * (screw_x - 42)  # bracket face at the screw
    top_z = min(RING_TOP, z_b + (TAB_GAP + TAB_T) / math.cos(math.radians(BRACKET_DEG)))  # tab top at the screw
    cb_z0 = top_z - M4_HEAD_INDENT  # flat head pocket floor
    for s in (-1, 1):
        y = s * SCREW_PITCH / 2
        screws.append(Pos(screw_x, y, (z_b - 5 + cb_z0) / 2) * Cylinder(M4_CLEAR_D / 2, cb_z0 - z_b + 5))
        screws.append(Pos(screw_x, y, cb_z0 + 30) * Cylinder(M4_HEAD_D / 2, 60))  # head pocket + driver access

    # legs run straight down to the ring bottom; where they stick out past the slim ring, small
    # plan-view triangles join the leg's outer corners to the ring edge. Front leg: both sides.
    # Side legs: both, the back one carries the pinch clamp.
    leg_ro = in_half_w + RIB_D

    def leg_triangle(sy):  # in the frame of a leg on +X; sy = which side (±Y)
        # corner exactly on the leg's corner; the 0.5 mm strip inside the leg gives a solid overlap
        pts = leg_tri_pts(sy)
        return Pos(0, 0, RING_BOT) * extrude(ccw(pts), amount=RING_TOP - RING_BOT)

    def leg_tri_pts(sy):
        t = math.radians(sy * FILLET_DEG)
        return [(leg_ro, sy * (rib / 2 - 0.5)), (leg_ro, sy * rib / 2),
                (ring_ro * math.cos(t), ring_ro * math.sin(t)),
                (in_half_w, sy * rib / 2), (in_half_w, sy * (rib / 2 - 0.5))]

    def root_wedge(sy):
        # fillet along the leg's side where it meets the ring top: a triangle in the leg's YZ plane run
        # radially along the leg, clipped to the ring + this side's leg triangle so it never overhangs
        g = ROOT_FILLET
        sec = ccw([(sy * (rib / 2 - 0.5), RING_TOP - 0.5), (sy * (rib / 2 + g), RING_TOP - 0.5),
                   (sy * (rib / 2 - 0.5), RING_TOP + g)])
        wedge = extrude(Plane.YZ.offset(in_half_w) * sec, amount=leg_ro - in_half_w)
        footprint = (Cylinder(ring_ro, 100) - Cylinder(in_half_w - 0.5, 100)
                     + Pos(0, 0, -50) * extrude(ccw(leg_tri_pts(sy)), amount=100))
        return wedge & footprint

    # pinch clamp on each side leg's back triangle, in that leg's frame. The M3 runs parallel to
    # the triangle's slanted face, in a boss hugging its outside; the boss continues past the
    # triangle tip onto the ring, and a slit square to the bolt splits it into a front ear (on the
    # triangle/leg) and a rear ear (on the ring). Head in the rear ear, nut in the front one.
    def pinch(sy, q):  # sy = sign of the leg's rear side; q = M4 tab end corner (leg frame) for the web
        t = math.radians(sy * FILLET_DEG)
        # leg corner, nudged 0.05 mm out: the boss face exactly coplanar with the triangle face
        # stalls OCC's boolean (minutes)
        a = (leg_ro + 0.05, sy * (rib / 2 + 0.05))
        b = (ring_ro * math.cos(t), ring_ro * math.sin(t))        # triangle tip on the ring
        hyp = math.hypot(b[0] - a[0], b[1] - a[1])
        d = ((b[0] - a[0]) / hyp, (b[1] - a[1]) / hyp)            # along the slanted face
        n = (d[1] * sy, -d[0] * sy)                                # outward normal
        ang = math.degrees(math.atan2(d[1], d[0]))
        off = BOSS_W / 2                                           # bolt line offset from the face

        def at(shape, t0, o=off):  # shape built along +X from x=0, laid along d from t0
            return Pos(a[0] + o * n[0] + t0 * d[0], a[1] + o * n[1] + t0 * d[1], bolt_z) * Rot(0, 0, ang) * shape

        boss_len = hyp + SLIT_W + EAR_L
        # boss plan in (t, outward) coords, laid on the slanted face. Local +Y of `at` is -sy * outward.
        # Extended inward past the ring, then trimmed on the legs' inner radius: the ears reach over
        # the ring and meet it on an arc
        ext = 20.0

        def face(pts_to):  # (t, outward) points -> CCW face in the `at` frame (local +Y = -sy * outward)
            pts = [(t_, -sy * o_) for t_, o_ in pts_to]
            area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
            return Polygon(*(pts if area > 0 else pts[::-1]), align=None)

        # web: straight outer edge from the ear's outer corner to the M4 tab's end corner, filled in
        # to the ring. Joined to the ear in 2D (a 3D union of the two stalls OCC), overlapping the
        # ear's last 2 mm (meeting at the corner only left a sliver + a non-manifold edge)
        to = lambda v: ((v[0] - a[0]) * d[0] + (v[1] - a[1]) * d[1], (v[0] - a[0]) * n[0] + (v[1] - a[1]) * n[1])
        plan = (face([(0, -ext), (boss_len, -ext), (boss_len, BOSS_W), (0, BOSS_W)])
                + face([to((0, 0)), (boss_len - 2, -ext), (boss_len - 2, BOSS_W), (boss_len, BOSS_W), to(q)]))
        prism = extrude(plan, amount=EAR_H)
        boss = at(Pos(0, 0, -EAR_H / 2) * prism, 0, 0)
        boss = boss - Cylinder(in_half_w, 100)  # arc ends where the cage legs' inner face is
        # slit: square to the bolt just past the triangle tip, through boss + ring only
        t_slit = hyp + 0.5 - SLIT_FWD                              # slit start, moved toward the head
        slit = at(Pos(SLIT_W / 2, 0, 0) * Box(SLIT_W, 2 * (BOSS_W + 25), EAR_H + 2), t_slit, 0)
        x_axis = Rot(0, 90, 0)
        # head on the front ear (pocket opens into free space past the leg corner); the nut drops
        # into a slot from the top of the rear ear
        t_head = M3_POCKET
        t_nut = t_slit + SLIT_W + NUT_WALL                       # nut seat, NUT_WALL behind the slit
        assert t_head + M3_LEN >= t_nut + 2.4 + 1, "M3_LEN too short to pass the nut"
        hole_len = boss_len + BOSS_W + 20                         # through both ends: any screw length
        hole = at(x_axis * Cylinder(M3_CLEAR_D / 2, hole_len, align=(Align.CENTER, Align.CENTER, Align.MIN)), -10)
        head = at(x_axis * Cylinder(M3_HEAD_D / 2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN)), t_head - 30)  # round counterbore, head flush
        # nut slot: drop-in from the top, half-hex bottom so the nut seats on two faces centred
        # on the bolt (flats vertical, corner down)
        af = M3_NUT_AF + M3_NUT_CLEAR
        nut_hex = Rot(0, 90, 0) * extrude(RegularPolygon(af / math.sqrt(3), 6), amount=M3_NUT_T)  # extrudes +X, corners ±Z
        nut = at(nut_hex + Pos(M3_NUT_T / 2, 0, EAR_H / 2) * Box(M3_NUT_T, af - 0.02, EAR_H), t_nut)  # -0.02: walls not coplanar with the hex flats
        corner = (a[0] + BOSS_W * n[0] + boss_len * d[0], a[1] + BOSS_W * n[1] + boss_len * d[1])
        return boss, [slit, hole, head, nut], corner

    bolt_z = RING_BOT + EAR_H / 2
    hoops = hoop_x + hoop_y + Rot(0, 0, 180) * (leg_triangle(1) + leg_triangle(-1))
    for az in (90, 180, 270):  # front + side legs (the rear one lands on the tab)
        hoops = hoops + Rot(0, 0, az) * (root_wedge(1) + root_wedge(-1))
    adds, cuts = [], []
    for az, sy_rear in ((90, -1), (270, 1)):
        r = Rot(0, 0, az)
        hoops = hoops + r * (leg_triangle(1) + leg_triangle(-1))
        c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
        q = (TAB_END, math.copysign(half_w, s_))                  # M4 tab end corner on this side (world)
        boss, holes, corner = pinch(sy_rear, (q[0] * c + q[1] * s_, -q[0] * s_ + q[1] * c))
        boss = r * boss
        # web top slopes from the ear top at its outer corner down to the tab/ring top at q
        p = (corner[0] * c - corner[1] * s_, corner[0] * s_ + corner[1] * c)
        ear_top = RING_BOT + EAR_H
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        k = (ear_top - RING_TOP) / L
        u = ((q[0] - p[0]) / L, (q[1] - p[1]) / L)
        slope = Plane(origin=(p[0], p[1], ear_top), z_dir=(k * u[0], k * u[1], 1))
        # keep the web out of the bracket gap entirely: the tab owns that region
        boss = ((boss - slope * Pos(0, 0, 100) * Box(500, 500, 200)) & below_top) - Pos(100, 0, 0) * Box(200, gap_w, 200)
        adds.append(boss)
        cuts += [r * h for h in holes]

    body = ring + hoops - hug_bore - spin_bore - slot
    out = body + tab - spin_bore - slot - no_ring
    # rear leg: steep gusset from its outer face down onto the tab, so it doesn't start square
    # in the air; leg-wide, which keeps it clear of the M4 counterbores at y=±SCREW_PITCH/2
    run = REAR_GUSSET_H / math.tan(math.radians(REAR_GUSSET_DEG))
    gusset = extrude(Plane.XZ * Polygon((leg_ro - 0.5, RING_TOP - 1), (leg_ro + run, RING_TOP - 1),
                                        (leg_ro - 0.5, RING_TOP + REAR_GUSSET_H), align=None),
                     amount=rib / 2, both=True)
    out = out + gusset
    # ...and underneath: inside r=ring_ro the bracket gap is empty, so the leg's inner edge would
    # hang over it as an L; chamfer that underside up from the tab edge instead
    under = (x_cut - in_half_w + 0.2) * math.tan(math.radians(REAR_GUSSET_DEG))
    relief = extrude(Plane.XZ * Polygon((in_half_w - 0.6, RING_TOP - 0.01), (x_cut + 0.1, RING_TOP - 0.01),
                                        (in_half_w - 0.6, RING_TOP + under), align=None),
                     amount=rib / 2 + 1, both=True)
    out = out - relief
    for a_ in adds:
        out = out + a_
    for c in cuts:
        out = out - c
    for c in screws:
        out = out - c
    # floor trim
    s = FLOOR_S - FLOOR_MARGIN - 500
    return out & Pos(math.sin(a) * s, 0, math.cos(a) * s) * Rot(0, FLOOR_DEG, 0) * Box(1000, 1000, 1000)


def head_reference() -> Part:
    # decimated voxel remesh of menagerie base_4's head region (ref/remesh_head.py), display only
    return import_stl(__file__.rsplit("/", 1)[0] + "/ref/head_shell_8000.stl")


def build_print() -> Assembly:
    return (
        Assembly()
        .with_part("cage", cage(), color=Color(0.9, 0.9, 0.9))
        .assert_solid_count("cage", detail="above 1 a cut has severed the ring, a hoop leg or the thin back corner")
    )


def build_fit(clear: float = CLEAR, show_head: bool = False) -> Assembly:
    a = (
        Assembly()
        .with_part("cage", cage(clear=clear), color=Color(0.9, 0.9, 0.9))
        .with_part("spin_envelope", spin_envelope(), color=Color(0.3, 0.55, 0.85, 0.5))
        .assert_solid_count("cage", detail="above 1 a cut has severed the ring, a hoop leg or the thin back corner")
        .assert_no_interference("cage", "spin_envelope")
        # tightest spot: ring top edge (hugging the 87 mm housing) vs the 84 mm cap's bottom edge at the
        # rim, with the housing lip between them (~1.9 mm); hoops keep >= `clear`
        .assert_distance("cage", "spin_envelope", min_mm=1.5)
    )
    if show_head:
        a = a.with_part("head_ref", head_reference(), color=Color(0.6, 0.6, 0.65, 0.6))
    return a


def build_view() -> Assembly:
    """Viewer only (khana view ...:build_view): adds the head mesh, which `check` can't load, and the
    lying-down floor (transparent slab)."""
    n = (math.sin(math.radians(FLOOR_DEG)), math.cos(math.radians(FLOOR_DEG)))
    s = FLOOR_S + 0.5
    floor = Pos(n[0] * s, 0, n[1] * s) * Rot(0, FLOOR_DEG, 0) * Box(300, 200, 1)
    return build_fit(show_head=True).with_part("floor", floor, color=Color(0.2, 0.4, 1.0, 0.25))


assembly = build_fit
