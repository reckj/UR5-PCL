"""
UR5e four-bar parallel gripper — Lynxmotion LSS servo + LilyGO T-CAN485
=======================================================================
Fusion 360 API build script.  Regenerates the whole model from the parameter
block below into the ACTIVE design (deletes previously generated components
first, so it can be re-run after editing parameters).

Run it from Fusion: Utilities > Add-Ins > Scripts (point at this file), or
exec() it from the Fusion MCP.  All values in mm; the API works in cm, hence MM.

Coordinate frame (= UR tool flange frame, tool mounted rotated 45 deg so the
M6 bolts sit on the X/Y axes):
    origin  = centre of the UR tool flange face
    +Z      = away from the robot, toward the jaws
    X       = jaw opening direction (jaws close toward x = 0)
    +Y      = "forward" (jaws sit forward of the linkage pivot line)

Layer stack above the flange (z):
    0..8       body floor (flange plate)            8..29  electronics bay (T-CAN485)
    29..32     lid                                  32..46.8 tower (servo tower + palm support)
    46.8..50.8 palm plate (bolts to servo top SES pattern + tower posts)
    51.3..57.3 gear layer   (2 x 20T m2 spur gears, right one on the servo horn)
    57.3..63.8 link layer   (drive arms + follower links)
    64.3..70.3 carrier layer (finger carriers + jaws, jaws reach down to 50.8)
    70.8..73.8 cover plate

ASSUMPTIONS TO VERIFY WITH CALIPERS: horn_top (6.5), horn hole clocking,
T-CAN485 mounting-hole positions, UR dowel-pin angle.
"""
import adsk.core, adsk.fusion, math, traceback

MM = 0.1
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# ----------------------------------------------------------------- parameters
P = dict(
    # UR tool flange ISO 9409-1-50-4-M6
    pcd=50.0, m6_clear=6.6, bolt_angle0=0.0,       # bolts on X/Y axes (tool rotated 45 deg on flange)
    pin_d=6.0, pin_depth=6.0, pin_angle=45.0,      # optional dowel, blind hole from flange face
    # body / electronics bay
    body_x=48.0, body_y0=-74.0, body_y1=54.0,      # outer footprint 96 x 128
    floor=8.0, cavity_h=21.0, wall=3.0,
    post=7.0, insert_d=4.0, insert_depth=6.0, m3_clear=3.4, m3_head_d=6.0,
    pcb_x0=-37.0, pcb_w=35.0, pcb_y0=-44.5, pcb_l=80.0, pcb_conn_over=13.5,
    pcb_hole_inset=3.0, standoff_d=6.0, standoff_h=7.0, standoff_hole=2.6,
    cable_notch_w=12.0, cable_notch_h=8.0, cable_notch_x=20.0,
    end_window_h=12.0,
    # Lynxmotion LSS servo: case 51 x 26 x 38.3, spline 12.7 from front end, centred in width
    servo_l=51.0, servo_w=26.0, servo_h=38.3, spline_from_front=12.7,
    ses2_offset=25.4, ses_r=16.66 / 2, ses_hole_d=2.3, ses_cb_d=4.6, ses_cb_depth=2.0,
    servo_clear=0.3, servo_lift=0.5,
    horn_d=24.0, horn_top=6.5,                     # horn top face above servo top face (VERIFY)
    horn_hole_d=2.3, horn_hole_clock=45.0, horn_center_hole=7.0, idler_hole_d=28.0,
    # lid + tower
    lid_t=3.0, tower_y1=-8.0, tower_wall=4.0, tower_post=8.0,
    # palm / cover
    palm_x=56.0, palm_t=4.0, palm_horn_hole=30.0, cover_t=3.0,
    # linkage (four-bar parallel gripper, gear-coupled)
    d=40.0,            # drive pivot spacing = gear centre distance (m*z = 2*20)
    s=28.0,            # drive pivot -> follower pivot spacing along X
    L=35.0,            # link length (pivot to pivot)
    link_w=8.0, link_t=6.0, gap=0.5,
    phi_closed=-20.0, phi_open=32.0, phi_pose=0.0,   # link angle from +Y, +ve = outward
    pivot_y=-30.0,
    gear_m=2.0, gear_z=20, pressure_deg=20.0, backlash=0.15,
    sleeve_od=6.0, bore=6.4, carrier_w=10.0, jaw_t=6.0, jaw_len=30.0, jaw_gap_y=2.0,
    boss_d=8.0, boss_clear=0.3, bush_od=5.0, bush_clear=0.4,
)

GEN_NAMES = ['01 Body (flange + bay)', '02 Upper (lid + tower)', '03 Palm', '04 Hub R (drive, on horn)',
             '05 Hub L', '06 Follower R', '07 Follower L', '08 Carrier R + jaw', '09 Carrier L + jaw',
             '10 Cover', '11 Pivot sleeves x3', '90 REF envelopes', '12 Pivot bushings x4']


# ----------------------------------------------------------------- helpers
def V(mm):
    return adsk.core.ValueInput.createByReal(mm * MM)


class Part:
    """One gripper part. A component in an Assembly document; in a Part document
    (single component only) it falls back to prefixed bodies in the root."""

    def __init__(self, root, name):
        self.name = name
        try:
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            self.comp = occ.component
            self.prefix = ''
        except RuntimeError:
            self.comp = root
            self.prefix = name + ' / '


def _comp(c):
    return getattr(c, 'comp', c)


class S:
    """Sketch on an XY plane at height z, with mm helpers."""

    def __init__(self, comp, z, name=None):
        comp = _comp(comp)
        inp = comp.constructionPlanes.createInput()
        inp.setByOffset(comp.xYConstructionPlane, V(z))
        plane = comp.constructionPlanes.add(inp)
        plane.isLightBulbOn = False
        self.sk = comp.sketches.add(plane)
        if name:
            self.sk.name = name
        self.z = z

    def p(self, x, y):
        return self.sk.modelToSketchSpace(adsk.core.Point3D.create(x * MM, y * MM, self.z * MM))

    def rect(self, x0, y0, x1, y1):
        self.sk.sketchCurves.sketchLines.addTwoPointRectangle(self.p(x0, y0), self.p(x1, y1))

    def circle(self, cx, cy, d):
        self.sk.sketchCurves.sketchCircles.addByCenterRadius(self.p(cx, cy), d / 2 * MM)

    def poly(self, pts):
        lines = self.sk.sketchCurves.sketchLines
        self.sk.isComputeDeferred = True
        n = len(pts)
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            lines.addByTwoPoints(self.p(*a), self.p(*b))
        self.sk.isComputeDeferred = False

    def slot(self, x0, y0, x1, y1, w):
        """Axis-aligned stadium from (x0,y0) to (x1,y1), width w."""
        r = w / 2
        if abs(x1 - x0) < 1e-9:      # vertical
            self.rect(x0 - r, min(y0, y1), x0 + r, max(y0, y1))
        else:                        # horizontal
            self.rect(min(x0, x1), y0 - r, max(x0, x1), y0 + r)
        self.circle(x0, y0, w)
        self.circle(x1, y1, w)


def extrude(comp, s, h, op, target=None, name=None):
    prefix = getattr(comp, 'prefix', '')
    comp = _comp(comp)
    sk = s.sk
    profs = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        profs.add(sk.profiles.item(i))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, V(h))
    if target is not None and op != NEW:
        inp.participantBodies = [target]
    f = ext.add(inp)
    if name:
        f.name = name
        if op == NEW and f.bodies.count:
            f.bodies.item(0).name = prefix + name
    return f


def new_comp(root, name):
    return Part(root, name)


def cleanup(des, root):
    """Remove everything generated by a previous run."""
    for i in range(root.occurrences.count - 1, -1, -1):
        occ = root.occurrences.item(i)
        if occ.component.name in GEN_NAMES:
            occ.deleteMe()
    ours = [b for b in root.bRepBodies if ' / ' in b.name and b.name.split(' / ')[0] in GEN_NAMES]
    if ours or root.bRepBodies.count == 0:
        tl = des.timeline
        for i in range(tl.count - 1, -1, -1):
            try:
                tl.item(i).entity.deleteMe()
            except Exception:
                pass


def gear_pts(cx, cy, m, z, pa_deg, rot, backlash):
    rp = m * z / 2.0
    pa = math.radians(pa_deg)
    rb = rp * math.cos(pa)
    ra = rp + m
    rf = rp - 1.25 * m
    inv_pa = math.tan(pa) - pa
    hp = math.pi / (2 * z)
    bl = backlash / (2 * rp)

    def inv(r):
        a = math.acos(min(1.0, rb / r))
        return math.tan(a) - a

    n_inv = 5
    pts = []
    for k in range(z):
        tc = rot + 2 * math.pi * k / z
        psi_b = hp + inv_pa - bl
        pts.append((rf, tc - psi_b))
        for i in range(n_inv + 1):
            r = rb + (ra - rb) * i / n_inv
            pts.append((r, tc - (hp + inv_pa - inv(r) - bl)))
        for i in range(n_inv, -1, -1):
            r = rb + (ra - rb) * i / n_inv
            pts.append((r, tc + (hp + inv_pa - inv(r) - bl)))
        pts.append((rf, tc + psi_b))
    return [(cx + r * math.cos(t), cy + r * math.sin(t)) for r, t in pts]


# ----------------------------------------------------------------- derived geometry
def derive(P):
    D = {}
    D['spline_x'] = P['d'] / 2
    D['py'] = P['pivot_y']
    D['e'] = P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_closed']))   # carrier pivot -> jaw face
    D['z_floor'] = P['floor']
    D['z_body_top'] = P['floor'] + P['cavity_h']
    D['z_lid_top'] = D['z_body_top'] + P['lid_t']
    D['z_servo_bot'] = P['floor'] + P['servo_lift']
    D['z_servo_top'] = D['z_servo_bot'] + P['servo_h']
    D['tower_h'] = D['z_servo_top'] - D['z_lid_top']
    D['z_palm0'] = D['z_servo_top']
    D['z_palm1'] = D['z_palm0'] + P['palm_t']
    D['z_gear0'] = D['z_palm1'] + P['gap']
    D['z_gear1'] = D['z_gear0'] + P['link_t']
    D['z_arm0'] = D['z_gear1']
    D['z_link0'] = D['z_gear1'] + P['gap']
    D['z_link1'] = D['z_link0'] + P['link_t']
    D['z_car0'] = D['z_link1'] + P['gap']
    D['z_car1'] = D['z_car0'] + P['link_t']
    D['z_cov0'] = D['z_car1'] + P['gap']
    D['z_cov1'] = D['z_cov0'] + P['cover_t']
    sx = D['spline_x']
    D['servo_x0'] = sx - P['servo_w'] / 2
    D['servo_x1'] = sx + P['servo_w'] / 2
    D['servo_y1'] = D['py'] + P['spline_from_front']
    D['servo_y0'] = D['servo_y1'] - P['servo_l']
    D['ses2'] = (sx, D['py'] - P['ses2_offset'])
    D['tower_y0'] = P['body_y0']
    D['post_x'] = P['body_x'] - P['wall'] - P['post'] / 2             # body posts hug the side walls
    D['post_y_back'] = P['body_y0'] + P['wall'] + P['post'] / 2
    D['post_y_front'] = P['body_y1'] - P['wall'] - P['post'] / 2
    D['post_y_mid'] = 0.0                                             # clear of tower (+Y wall at -8)
    D['tpost_y_front'] = P['tower_y1'] - P['tower_wall'] - P['tower_post'] / 2 + 2.0
    # pose
    phi = math.radians(P['phi_pose'])
    D['arm_end_y'] = D['py'] + P['L'] * math.cos(phi)
    D['arm_end_dx'] = P['L'] * math.sin(phi)
    D['opening_closed'] = 2 * (P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_closed'])) - D['e'])
    D['opening_open'] = 2 * (P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_open'])) - D['e'])
    D['opening_pose'] = 2 * (P['d'] / 2 + D['arm_end_dx'] - D['e'])
    return D


# ----------------------------------------------------------------- parts
def build_body(root, P, D):
    c = new_comp(root, GEN_NAMES[0])
    bx, y0, y1 = P['body_x'], P['body_y0'], P['body_y1']
    s = S(c, 0, 'outer')
    s.rect(-bx, y0, bx, y1)
    body = extrude(c, s, D['z_body_top'], NEW, name='Body').bodies.item(0)
    # bay cavity
    w = P['wall']
    s = S(c, D['z_floor'], 'cavity')
    s.rect(-bx + w, y0 + w, bx - w, y1 - w)
    extrude(c, s, P['cavity_h'], CUT, body, 'cavity')
    # corner + mid posts with M3 inserts
    posts = []
    for sx in (-1, 1):
        for py in (D['post_y_back'], D['post_y_front'], D['post_y_mid']):
            posts.append((sx * D['post_x'], py))
    s = S(c, D['z_floor'], 'posts')
    h = P['post'] / 2
    for (x, y) in posts:
        s.rect(x - h, y - h, x + h, y + h)
    extrude(c, s, P['cavity_h'], JOIN, body, 'posts')
    s = S(c, D['z_body_top'], 'post inserts')
    for (x, y) in posts:
        s.circle(x, y, P['insert_d'])
    extrude(c, s, -P['insert_depth'], CUT, body, 'post inserts M3')
    # UR flange: 4 x M6 clearance through floor, dowel blind hole
    s = S(c, 0, 'M6 holes')
    r = P['pcd'] / 2
    for k in range(4):
        a = math.radians(P['bolt_angle0'] + 90 * k)
        s.circle(r * math.cos(a), r * math.sin(a), P['m6_clear'])
    extrude(c, s, P['floor'], CUT, body, 'M6 x4 clearance')
    s = S(c, 0, 'dowel')
    a = math.radians(P['pin_angle'])
    s.circle(r * math.cos(a), r * math.sin(a), P['pin_d'])
    extrude(c, s, P['pin_depth'], CUT, body, 'dowel pin hole')
    # idler-horn through hole under servo
    s = S(c, 0, 'idler hole')
    s.circle(D['spline_x'], D['py'], P['idler_hole_d'])
    extrude(c, s, P['floor'], CUT, body, 'idler horn clearance')
    # PCB standoffs
    ins = P['pcb_hole_inset']
    so = [(P['pcb_x0'] + ins, P['pcb_y0'] + ins), (P['pcb_x0'] + P['pcb_w'] - ins, P['pcb_y0'] + ins),
          (P['pcb_x0'] + ins, P['pcb_y0'] + P['pcb_l'] - ins),
          (P['pcb_x0'] + P['pcb_w'] - ins, P['pcb_y0'] + P['pcb_l'] - ins)]
    s = S(c, D['z_floor'], 'standoffs')
    for (x, y) in so:
        s.circle(x, y, P['standoff_d'])
    extrude(c, s, P['standoff_h'], JOIN, body, 'PCB standoffs')
    s = S(c, D['z_floor'] + P['standoff_h'], 'standoff holes')
    for (x, y) in so:
        s.circle(x, y, P['standoff_hole'])
    extrude(c, s, -P['standoff_h'], CUT, body, 'standoff holes')
    # cable notch (tool-connector cable) in +Y end wall, top edge
    s = S(c, D['z_body_top'] - P['cable_notch_h'], 'cable notch')
    cx = P['cable_notch_x']
    s.rect(cx - P['cable_notch_w'] / 2, y1 - w - 0.5, cx + P['cable_notch_w'] / 2, y1 + 0.5)
    extrude(c, s, P['cable_notch_h'] + 0.5, CUT, body, 'cable notch')
    # end-wall windows over the PCB ends (USB-C / terminal access)
    s = S(c, D['z_floor'] + P['standoff_h'] - 1.0, 'end windows')
    s.rect(P['pcb_x0'], y0 - 0.5, P['pcb_x0'] + P['pcb_w'], y0 + w + 0.5)
    s.rect(P['pcb_x0'], y1 - w - 0.5, P['pcb_x0'] + P['pcb_w'], y1 + 0.5)
    extrude(c, s, P['end_window_h'], CUT, body, 'end windows')
    return c


def build_upper(root, P, D):
    c = new_comp(root, GEN_NAMES[1])
    bx, y0, y1 = P['body_x'], P['body_y0'], P['body_y1']
    s = S(c, D['z_body_top'], 'lid')
    s.rect(-bx, y0, bx, y1)
    body = extrude(c, s, P['lid_t'], NEW, name='Upper').bodies.item(0)
    # lid screw holes (all 6 body posts)
    s = S(c, D['z_body_top'], 'lid holes')
    for sx in (-1, 1):
        for py in (D['post_y_back'], D['post_y_front'], D['post_y_mid']):
            s.circle(sx * D['post_x'], py, P['m3_clear'])
    extrude(c, s, P['lid_t'], CUT, body, 'lid M3 holes')
    # servo pocket through lid
    cl = P['servo_clear']
    s = S(c, D['z_body_top'], 'servo pocket')
    s.rect(D['servo_x0'] - cl, D['servo_y0'] - cl, D['servo_x1'] + cl, D['servo_y1'] + cl)
    extrude(c, s, P['lid_t'], CUT, body, 'servo pocket')
    # tower walls
    ty0, ty1, tw = D['tower_y0'], P['tower_y1'], P['tower_wall']
    s = S(c, D['z_lid_top'], 'tower outer')
    s.rect(-bx, ty0, bx, ty1)
    extrude(c, s, D['tower_h'], JOIN, body, 'tower')
    s = S(c, D['z_lid_top'], 'tower inner')
    s.rect(-bx + tw, ty0 + tw, bx - tw, ty1 - tw)
    extrude(c, s, D['tower_h'], CUT, body, 'tower hollow')
    # tower posts
    tp = P['tower_post'] / 2
    tposts = [(sx * D['post_x'], y) for sx in (-1, 1) for y in (D['post_y_back'], D['tpost_y_front'])]
    s = S(c, D['z_lid_top'], 'tower posts')
    for (x, y) in tposts:
        s.rect(x - tp, y - tp, x + tp, y + tp)
    extrude(c, s, D['tower_h'], JOIN, body, 'tower posts')
    # back posts: through hole for long M3 (palm -> tower -> lid -> body insert)
    s = S(c, D['z_servo_top'], 'tower through')
    for sx in (-1, 1):
        s.circle(sx * D['post_x'], D['post_y_back'], P['m3_clear'])
    extrude(c, s, -(D['tower_h'] + P['lid_t']), CUT, body, 'tower through holes')
    # front posts: M3 insert from top
    s = S(c, D['z_servo_top'], 'tower inserts')
    for sx in (-1, 1):
        s.circle(sx * D['post_x'], D['tpost_y_front'], P['insert_d'])
    extrude(c, s, -P['insert_depth'], CUT, body, 'tower insert holes')
    return c


def build_palm(root, P, D):
    c = new_comp(root, GEN_NAMES[2])
    px, y0, y1 = P['palm_x'], D['tower_y0'], P['tower_y1']
    s = S(c, D['z_palm0'], 'palm')
    s.rect(-px, y0, px, y1)
    body = extrude(c, s, P['palm_t'], NEW, name='Palm').bodies.item(0)
    sx, py = D['spline_x'], D['py']
    s = S(c, D['z_palm0'], 'palm holes')
    s.circle(sx, py, P['palm_horn_hole'])                        # horn clearance
    for (x, y) in [(-sx, py), (sx + P['s'], py), (-sx - P['s'], py)]:
        s.circle(x, y, P['insert_d'])                            # pivot screw inserts (from top)
    for ssx in (-1, 1):
        s.circle(ssx * D['post_x'], D['post_y_back'], P['m3_clear'])
        s.circle(ssx * D['post_x'], D['tpost_y_front'], P['m3_clear'])
    # servo top SES pattern #2: 4 x 2-56 on 16.66 circle + M3 centre
    cx, cy = D['ses2']
    for k in range(4):
        a = math.radians(90 * k)
        s.circle(cx + P['ses_r'] * math.cos(a), cy + P['ses_r'] * math.sin(a), P['ses_hole_d'])
    s.circle(cx, cy, P['m3_clear'])
    extrude(c, s, P['palm_t'], CUT, body, 'palm holes')
    s = S(c, D['z_palm1'], 'palm counterbores')
    for k in range(4):
        a = math.radians(90 * k)
        s.circle(cx + P['ses_r'] * math.cos(a), cy + P['ses_r'] * math.sin(a), P['ses_cb_d'])
    s.circle(cx, cy, P['m3_head_d'])
    extrude(c, s, -P['ses_cb_depth'], CUT, body, 'SES counterbores')
    return c


def build_hub(root, P, D, right=True):
    c = new_comp(root, GEN_NAMES[3] if right else GEN_NAMES[4])
    sgn = 1 if right else -1
    gx, gy = sgn * D['spline_x'], D['py']
    rot = 0.0 if right else math.pi / P['gear_z']
    s = S(c, D['z_gear0'], 'gear')
    s.poly(gear_pts(gx, gy, P['gear_m'], P['gear_z'], P['pressure_deg'], rot, P['backlash']))
    body = extrude(c, s, P['link_t'], NEW, name=('Hub R' if right else 'Hub L')).bodies.item(0)
    # drive arm (link layer, sits directly on the gear)
    ex, ey = gx + sgn * D['arm_end_dx'], D['arm_end_y']
    s = S(c, D['z_arm0'], 'arm')
    s.slot(gx, gy, ex, ey, P['link_w'])
    extrude(c, s, D['z_link1'] - D['z_arm0'], JOIN, body, 'arm')
    if right:
        # boss up to the cover (radial support of the cantilevered drive hub)
        s = S(c, D['z_gear1'], 'boss')
        s.circle(gx, gy, P['boss_d'])
        extrude(c, s, D['z_cov1'] - D['z_gear1'], JOIN, body, 'boss')
        # horn pocket from below + horn screw holes + centre recess for horn screw head
        depth = (D['z_servo_top'] + P['horn_top']) - D['z_gear0']
        if depth > 0.2:
            s = S(c, D['z_gear0'], 'horn pocket')
            s.circle(gx, gy, P['horn_d'] + 0.3)
            extrude(c, s, depth, CUT, body, 'horn pocket')
        s = S(c, D['z_gear0'], 'horn holes')
        for k in range(4):
            a = math.radians(P['horn_hole_clock'] + 90 * k)
            s.circle(gx + P['ses_r'] * math.cos(a), gy + P['ses_r'] * math.sin(a), P['horn_hole_d'])
        s.circle(gx, gy, P['horn_center_hole'])
        extrude(c, s, P['link_t'], CUT, body, 'horn screw holes')
    else:
        s = S(c, D['z_gear0'], 'bore')
        s.circle(gx, gy, P['bore'])
        extrude(c, s, D['z_link1'] - D['z_gear0'], CUT, body, 'pivot bore')
    # arm end: M3 insert hole for the carrier screw
    s = S(c, D['z_link1'], 'arm end')
    s.circle(ex, ey, P['insert_d'])
    extrude(c, s, -(D['z_link1'] - D['z_arm0']), CUT, body, 'arm end insert')
    return c


def build_follower(root, P, D, right=True):
    c = new_comp(root, GEN_NAMES[5] if right else GEN_NAMES[6])
    sgn = 1 if right else -1
    fx, fy = sgn * (D['spline_x'] + P['s']), D['py']
    ex, ey = fx + sgn * D['arm_end_dx'], D['arm_end_y']
    s = S(c, D['z_link0'], 'follower')
    s.slot(fx, fy, ex, ey, P['link_w'])
    body = extrude(c, s, P['link_t'], NEW, name=('Follower R' if right else 'Follower L')).bodies.item(0)
    s = S(c, D['z_link0'], 'follower holes')
    s.circle(fx, fy, P['bore'])
    s.circle(ex, ey, P['insert_d'])
    extrude(c, s, P['link_t'], CUT, body, 'follower holes')
    return c


def build_carrier(root, P, D, right=True):
    c = new_comp(root, GEN_NAMES[7] if right else GEN_NAMES[8])
    sgn = 1 if right else -1
    ix = sgn * (D['spline_x'] + D['arm_end_dx'])          # inner (drive arm) pivot
    ox = ix + sgn * P['s']                                 # outer (follower) pivot
    y = D['arm_end_y']
    s = S(c, D['z_car0'], 'carrier')
    s.slot(ix, y, ox, y, P['carrier_w'])
    body = extrude(c, s, P['link_t'], NEW, name=('Carrier R' if right else 'Carrier L')).bodies.item(0)
    xj_in = ix - sgn * D['e']                              # jaw inner (gripping) face
    xj_out = xj_in + sgn * P['jaw_t']
    yj0 = y + P['carrier_w'] / 2 + P['jaw_gap_y']
    yj1 = yj0 + P['jaw_len']
    # tongue (carrier layer) bridging carrier -> jaw
    s = S(c, D['z_car0'], 'tongue')
    s.rect(min(xj_in, ix + sgn * 5), y - P['carrier_w'] / 2, max(xj_in, ix + sgn * 5), yj0 + 0.5)
    extrude(c, s, P['link_t'], JOIN, body, 'tongue')
    # jaw block, from palm top level up to carrier top
    s = S(c, D['z_palm1'], 'jaw')
    s.rect(min(xj_in, xj_out), yj0, max(xj_in, xj_out), yj1)
    extrude(c, s, D['z_car1'] - D['z_palm1'], JOIN, body, 'jaw')
    # pivot holes run on Ø5 bushings (screw clamps bushing to link, carrier turns on bushing)
    s = S(c, D['z_car0'], 'carrier holes')
    s.circle(ix, y, P['bush_od'] + P['bush_clear'])
    s.circle(ox, y, P['bush_od'] + P['bush_clear'])
    extrude(c, s, P['link_t'], CUT, body, 'carrier pivot holes')
    return c


def build_bushings(root, P, D):
    c = new_comp(root, GEN_NAMES[12])
    h = P['gap'] + P['link_t'] + 0.1
    n = 0
    for sgn in (1, -1):
        ix = sgn * (D['spline_x'] + D['arm_end_dx'])
        for x in (ix, ix + sgn * P['s']):
            n += 1
            s = S(c, D['z_link1'], 'bushing %d' % n)
            s.circle(x, D['arm_end_y'], P['bush_od'])
            body = extrude(c, s, h, NEW, name='Bushing %d' % n).bodies.item(0)
            s = S(c, D['z_link1'], 'bushing bore %d' % n)
            s.circle(x, D['arm_end_y'], P['m3_clear'])
            extrude(c, s, h, CUT, body, 'bushing bore')
    return c


def build_cover(root, P, D):
    c = new_comp(root, GEN_NAMES[9])
    px, y0, y1 = P['palm_x'], D['tower_y0'], P['tower_y1']
    s = S(c, D['z_cov0'], 'cover')
    s.rect(-px, y0, px, y1)
    body = extrude(c, s, P['cover_t'], NEW, name='Cover').bodies.item(0)
    sx, py = D['spline_x'], D['py']
    s = S(c, D['z_cov0'], 'cover holes')
    s.circle(sx, py, P['boss_d'] + 2 * P['boss_clear'])
    for (x, y) in [(-sx, py), (sx + P['s'], py), (-sx - P['s'], py)]:
        s.circle(x, y, P['m3_clear'])
    extrude(c, s, P['cover_t'], CUT, body, 'cover holes')
    return c


def build_sleeves(root, P, D):
    c = new_comp(root, GEN_NAMES[10])
    sx, py = D['spline_x'], D['py']
    h = D['z_cov0'] - D['z_palm1']
    for i, (x, y) in enumerate([(-sx, py), (sx + P['s'], py), (-sx - P['s'], py)]):
        s = S(c, D['z_palm1'], 'sleeve %d' % i)
        s.circle(x, y, P['sleeve_od'])
        body = extrude(c, s, h, NEW, name='Sleeve %d' % (i + 1)).bodies.item(0)
        s = S(c, D['z_palm1'], 'sleeve bore %d' % i)
        s.circle(x, y, P['m3_clear'])
        extrude(c, s, h, CUT, body, 'sleeve bore')
    return c


def build_ref(root, P, D):
    c = new_comp(root, GEN_NAMES[11])
    # servo case
    s = S(c, D['z_servo_bot'], 'servo')
    s.rect(D['servo_x0'], D['servo_y0'], D['servo_x1'], D['servo_y1'])
    extrude(c, s, P['servo_h'], NEW, name='REF LSS servo case')
    sx, py = D['spline_x'], D['py']
    s = S(c, D['z_servo_top'], 'spline')
    s.circle(sx, py, 6.0)
    extrude(c, s, 3.0, NEW, name='REF spline boss')
    s = S(c, D['z_servo_top'] + 3.0, 'horn')
    s.circle(sx, py, P['horn_d'])
    extrude(c, s, P['horn_top'] - 3.0, NEW, name='REF horn (verify height)')
    s = S(c, D['z_servo_bot'], 'idler')
    s.circle(sx, py, P['horn_d'])
    extrude(c, s, -5.0, NEW, name='REF idler horn')
    # T-CAN485
    z = D['z_floor'] + P['standoff_h']
    s = S(c, z, 'pcb')
    s.rect(P['pcb_x0'], P['pcb_y0'], P['pcb_x0'] + P['pcb_w'], P['pcb_y0'] + P['pcb_l'])
    extrude(c, s, 1.6, NEW, name='REF T-CAN485 PCB 80x35')
    s = S(c, z + 1.6, 'conn')
    s.rect(P['pcb_x0'], P['pcb_y0'] - P['pcb_conn_over'], P['pcb_x0'] + P['pcb_w'], P['pcb_y0'] + 8)
    s.rect(P['pcb_x0'], P['pcb_y0'] + P['pcb_l'] - 8, P['pcb_x0'] + P['pcb_w'],
           P['pcb_y0'] + P['pcb_l'] + P['pcb_conn_over'])
    extrude(c, s, 10.5, NEW, name='REF terminals / USB envelope')
    # UR flange
    s = S(c, 0, 'ur flange')
    s.circle(0, 0, 63.0)
    extrude(c, s, -6.0, NEW, name='REF UR5e tool flange')
    return c


# ----------------------------------------------------------------- main
def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        des = adsk.fusion.Design.cast(app.activeProduct)
        root = des.rootComponent
        cleanup(des, root)
        D = derive(P)
        build_body(root, P, D)
        build_upper(root, P, D)
        build_palm(root, P, D)
        build_hub(root, P, D, True)
        build_hub(root, P, D, False)
        build_follower(root, P, D, True)
        build_follower(root, P, D, False)
        build_carrier(root, P, D, True)
        build_carrier(root, P, D, False)
        build_cover(root, P, D)
        build_sleeves(root, P, D)
        build_bushings(root, P, D)
        build_ref(root, P, D)
        msg = ('gripper built: opening closed=%.1f  open=%.1f  pose=%.1f mm; jaw centre y=%.1f; '
               'fingertip z=%.1f; e=%.2f' % (D['opening_closed'], D['opening_open'], D['opening_pose'],
                                             D['arm_end_y'] + P['carrier_w'] / 2 + P['jaw_gap_y'] + P['jaw_len'] / 2,
                                             D['z_car1'], D['e']))
        print(msg)
        return msg
    except Exception:
        print('FAILED:\n' + traceback.format_exc())
        raise
