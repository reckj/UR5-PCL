"""
UR5e four-bar parallel gripper v2 — side-mount (flange on the XZ plane)
=====================================================================
Lynxmotion LSS servo + LilyGO T-CAN485, Fusion 360 API build script.
Regenerates the whole model from the parameter block into the ACTIVE design
(previous generated parts are deleted first).  All values in mm.

Gripper frame (G):
    X = jaw opening axis (jaws close toward x = 0)
    Y = forward, toward the fingers  (= robot tool Z axis)
    Z = servo spline axis, "up" over the linkage stack
The UR tool flange face lies in the XZ plane at y = y_back, centred on
(0, y_back, z_fl) where z_fl is the mid-height of the jaw pads, so the TCP sits
on the flange axis.  Tool frame: X_t = X_G, Y_t = -Z_G, Z_t = Y_G.

Layout along Y (back → front):
    y_back .. y_back+33   01 Body: flange plate (8) + electronics bay, open to +Y
                          T-CAN485 standing on edge (plane XZ, long axis X)
    -78 .. -74            02 Upper: lid plate closing the bay  (+ tower in front)
    -74 .. -8             tower box around the servo, palm on top, linkage above
    12 .. 42              jaw pads
Layer stack in Z unchanged from v1 (servo 8.5–46.8, palm, gears, links,
carriers, cover 70.8–73.8).

ASSUMPTIONS TO VERIFY: horn_top (6.5), horn hole clocking, T-CAN485 hole
pattern / which edge carries USB-C & terminals, UR dowel angle.
"""
import adsk.core, adsk.fusion, math, traceback

MM = 0.1
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# ----------------------------------------------------------------- parameters
P = dict(
    # UR tool flange ISO 9409-1-50-4-M6 (bolts on the X_t / Y_t axes)
    pcd=50.0, m6_clear=6.6, bolt_angle0=0.0,
    pin_d=6.0, pin_depth=6.0, pin_angle=45.0,
    # body (bay) — plane XZ at the back
    body_x=58.0, body_depth=33.0, back_wall=8.0, wall=3.0, z_bot=3.0, z_top_margin=38.0,
    post=7.0, insert_d=4.0, insert_depth=6.0, m3_clear=3.4, m3_head_d=6.0,
    pcb_l=80.0, pcb_w=35.0, pcb_conn_over=13.5, pcb_t=1.6, pcb_comp_h=10.5,
    pcb_hole_inset=3.0, standoff_d=6.0, standoff_h=7.0, standoff_hole=2.6,
    window_h=30.0,
    # Lynxmotion LSS servo: case 51 x 26 x 38.3, spline 12.7 from front, centred in width
    servo_l=51.0, servo_w=26.0, servo_h=38.3, spline_from_front=12.7,
    ses2_offset=25.4, ses_r=16.66 / 2, ses_hole_d=2.3, ses_cb_d=4.6, ses_cb_depth=2.0,
    servo_clear=0.3, servo_lift=0.5,
    horn_d=24.0, horn_top=6.5,
    horn_hole_d=2.3, horn_hole_clock=45.0, horn_center_hole=7.0, idler_hole_d=28.0,
    # lid + tower
    lid_t=4.0, lid_front=-74.0, tower_x=48.0, tower_y1=-8.0, tower_wall=4.0,
    tower_floor=5.0, tower_post=8.0, tpost_y_back=-62.0,
    # palm / upstand / cover
    palm_x=56.0, palm_t=4.0, palm_horn_hole=30.0, cover_t=3.0,
    upstand_t=6.0, upstand_screw_x=(-32.0, 0.0, 32.0), cover_screw_x=(-40.0, 40.0),
    # linkage
    d=40.0, s=28.0, L=35.0, link_w=8.0, link_t=6.0, gap=0.5,
    phi_closed=-20.0, phi_open=32.0, phi_pose=0.0, pivot_y=-30.0,
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
    """Sketch on a plane parallel to XY (axis='z', at height z) or to XZ
    (axis='y', at depth y).  p(a, b) takes (x, y) resp. (x, z) in mm.
    self.dir = +1 if the plane normal points along +axis, so extrude() can
    treat a positive distance as 'toward +axis'."""

    def __init__(self, comp, v, name=None, axis='z'):
        comp = _comp(comp)
        base = comp.xYConstructionPlane if axis == 'z' else comp.xZConstructionPlane
        planes = comp.constructionPlanes
        plane = None
        for sign in (1, -1):
            inp = planes.createInput()
            inp.setByOffset(base, V(sign * v))
            pl = planes.add(inp)
            o = pl.geometry.origin
            coord = (o.z if axis == 'z' else o.y) / MM
            if abs(coord - v) < 1e-3:
                plane = pl
                break
            pl.deleteMe()
        if plane is None:
            raise RuntimeError('could not place plane %s=%g' % (axis, v))
        n = plane.geometry.normal
        self.dir = 1 if ((n.z if axis == 'z' else n.y) > 0) else -1
        plane.isLightBulbOn = False
        self.sk = comp.sketches.add(plane)
        if name:
            self.sk.name = name
        self.v = v
        self.axis = axis

    def p(self, a, b):
        if self.axis == 'z':
            pt = adsk.core.Point3D.create(a * MM, b * MM, self.v * MM)
        else:
            pt = adsk.core.Point3D.create(a * MM, self.v * MM, b * MM)
        return self.sk.modelToSketchSpace(pt)

    def rect(self, a0, b0, a1, b1):
        self.sk.sketchCurves.sketchLines.addTwoPointRectangle(self.p(a0, b0), self.p(a1, b1))

    def circle(self, ca, cb, d):
        self.sk.sketchCurves.sketchCircles.addByCenterRadius(self.p(ca, cb), d / 2 * MM)

    def poly(self, pts):
        lines = self.sk.sketchCurves.sketchLines
        self.sk.isComputeDeferred = True
        n = len(pts)
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            lines.addByTwoPoints(self.p(*a), self.p(*b))
        self.sk.isComputeDeferred = False

    def slot(self, a0, b0, a1, b1, w):
        r = w / 2
        if abs(a1 - a0) < 1e-9:
            self.rect(a0 - r, min(b0, b1), a0 + r, max(b0, b1))
        else:
            self.rect(min(a0, a1), b0 - r, max(a0, a1), b0 + r)
        self.circle(a0, b0, w)
        self.circle(a1, b1, w)


def extrude(comp, s, h, op, target=None, name=None):
    """Extrude all profiles of sketch s by h mm toward +axis (negative = -axis)."""
    prefix = getattr(comp, 'prefix', '')
    comp = _comp(comp)
    sk = s.sk
    profs = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        profs.add(sk.profiles.item(i))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, V(h * s.dir))
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
    for i in range(root.occurrences.count - 1, -1, -1):
        occ = root.occurrences.item(i)
        if occ.component.name in GEN_NAMES:
            occ.deleteMe()
    ours = [b for b in root.bRepBodies if ' / ' in b.name and b.name.split(' / ')[0] in GEN_NAMES]
    if ours or root.bRepBodies.count == 0:
        # Part document: everything in the timeline is ours -> wipe it in one operation
        # (deleting features one by one takes minutes and can stall Fusion)
        tl = des.timeline
        if tl.count:
            tl.markerPosition = 0
            tl.deleteAllAfterMarker()


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
    D['e'] = P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_closed']))
    # Z stack (unchanged from v1)
    D['z_servo_bot'] = P['z_bot'] + P['tower_floor'] + P['servo_lift']
    D['z_servo_top'] = D['z_servo_bot'] + P['servo_h']
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
    D['z_fl'] = (D['z_palm1'] + D['z_car1']) / 2            # flange axis = jaw mid height
    D['z_top'] = D['z_fl'] + P['z_top_margin']
    # Y layout
    D['y_lid0'] = P['lid_front'] - P['lid_t']               # -78
    D['y_back'] = D['y_lid0'] - P['body_depth']             # flange face
    D['y_cav0'] = D['y_back'] + P['back_wall']
    D['tower_y0'] = P['lid_front']
    # servo
    sx = D['spline_x']
    D['servo_x0'] = sx - P['servo_w'] / 2
    D['servo_x1'] = sx + P['servo_w'] / 2
    D['servo_y1'] = D['py'] + P['spline_from_front']
    D['servo_y0'] = D['servo_y1'] - P['servo_l']
    D['ses2'] = (sx, D['py'] - P['ses2_offset'])
    # body posts (columns along Y in the bay corners / mid)
    px = P['body_x'] - P['wall'] - P['post'] / 2
    zlo = P['z_bot'] + P['tower_floor'] + 1.0 + P['post'] / 2   # screw heads clear the tower floor
    zhi = D['z_top'] - P['wall'] - P['post'] / 2
    D['body_posts'] = [(-px, zlo), (px, zlo), (-px, zhi), (px, zhi), (-20.0, zlo), (-20.0, zhi)]
    # tower posts
    tx = P['tower_x'] - P['tower_wall'] - P['tower_post'] / 2 + 1.5
    D['tpost_x'] = tx
    D['tpost_y_front'] = P['tower_y1'] - P['tower_wall'] - P['tower_post'] / 2 + 2.0
    D['tower_posts'] = [(sx_ * tx, y) for sx_ in (-1, 1) for y in (P['tpost_y_back'], D['tpost_y_front'])]
    # upstand
    D['ups_y0'] = P['lid_front']
    D['ups_y1'] = P['lid_front'] + P['upstand_t']
    D['ups_screw_z'] = (D['z_palm1'] + D['z_cov0']) / 2
    # PCB (plane XZ, standing, centred on the flange axis)
    D['pcb_x0'] = -P['pcb_l'] / 2
    D['pcb_z0'] = D['z_fl'] - P['pcb_w'] / 2
    D['pcb_y'] = D['y_cav0'] + P['standoff_h']
    ins = P['pcb_hole_inset']
    D['pcb_holes'] = [(D['pcb_x0'] + ins, D['pcb_z0'] + ins), (-D['pcb_x0'] - ins, D['pcb_z0'] + ins),
                      (D['pcb_x0'] + ins, D['pcb_z0'] + P['pcb_w'] - ins),
                      (-D['pcb_x0'] - ins, D['pcb_z0'] + P['pcb_w'] - ins)]
    # pose
    phi = math.radians(P['phi_pose'])
    D['arm_end_y'] = D['py'] + P['L'] * math.cos(phi)
    D['arm_end_dx'] = P['L'] * math.sin(phi)
    D['opening_closed'] = 2 * (P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_closed'])) - D['e'])
    D['opening_open'] = 2 * (P['d'] / 2 + P['L'] * math.sin(math.radians(P['phi_open'])) - D['e'])
    D['opening_pose'] = 2 * (P['d'] / 2 + D['arm_end_dx'] - D['e'])
    D['jaw_y_mid'] = D['arm_end_y'] + P['carrier_w'] / 2 + P['jaw_gap_y'] + P['jaw_len'] / 2
    return D


# ----------------------------------------------------------------- parts
def build_body(root, P, D):
    c = new_comp(root, GEN_NAMES[0])
    bx, yb, zb, zt = P['body_x'], D['y_back'], P['z_bot'], D['z_top']
    w = P['wall']
    s = S(c, yb, 'outer', 'y')
    s.rect(-bx, zb, bx, zt)
    body = extrude(c, s, P['body_depth'], NEW, name='Body').bodies.item(0)
    # bay cavity, open toward +Y
    s = S(c, D['y_cav0'], 'cavity', 'y')
    s.rect(-bx + w, zb + w, bx - w, zt - w)
    extrude(c, s, P['body_depth'] - P['back_wall'] + 0.5, CUT, body, 'cavity')
    # screw posts (columns along Y) + inserts from the front rim
    h = P['post'] / 2
    s = S(c, D['y_cav0'], 'posts', 'y')
    for (x, z) in D['body_posts']:
        s.rect(x - h, z - h, x + h, z + h)
    extrude(c, s, P['body_depth'] - P['back_wall'], JOIN, body, 'posts')
    s = S(c, yb + P['body_depth'], 'post inserts', 'y')
    for (x, z) in D['body_posts']:
        s.circle(x, z, P['insert_d'])
    extrude(c, s, -P['insert_depth'], CUT, body, 'post inserts M3')
    # UR flange: 4 x M6 through the back wall, dowel blind
    zf = D['z_fl']
    r = P['pcd'] / 2
    s = S(c, yb, 'M6 holes', 'y')
    for k in range(4):
        a = math.radians(P['bolt_angle0'] + 90 * k)
        s.circle(r * math.cos(a), zf + r * math.sin(a), P['m6_clear'])
    extrude(c, s, P['back_wall'], CUT, body, 'M6 x4 clearance')
    s = S(c, yb, 'dowel', 'y')
    a = math.radians(P['pin_angle'])
    s.circle(r * math.cos(a), zf + r * math.sin(a), P['pin_d'])
    extrude(c, s, P['pin_depth'], CUT, body, 'dowel pin hole')
    # PCB standoffs on the back wall
    s = S(c, D['y_cav0'], 'standoffs', 'y')
    for (x, z) in D['pcb_holes']:
        s.circle(x, z, P['standoff_d'])
    extrude(c, s, P['standoff_h'], JOIN, body, 'PCB standoffs')
    s = S(c, D['pcb_y'], 'standoff holes', 'y')
    for (x, z) in D['pcb_holes']:
        s.circle(x, z, P['standoff_hole'])
    extrude(c, s, -P['standoff_h'], CUT, body, 'standoff holes')
    # side windows (USB-C / terminals / cable entry) through the ±X walls
    s = S(c, zf - P['window_h'] / 2, 'side windows')
    y0 = D['y_cav0'] + 1.0
    y1 = yb + P['body_depth'] - 1.0
    s.rect(bx - w - 0.5, y0, bx + 0.5, y1)
    s.rect(-bx - 0.5, y0, -bx + w + 0.5, y1)
    extrude(c, s, P['window_h'], CUT, body, 'side windows')
    return c


def build_upper(root, P, D):
    c = new_comp(root, GEN_NAMES[1])
    bx, zb, zt = P['body_x'], P['z_bot'], D['z_top']
    # lid plate (closes the bay) — plane XZ
    s = S(c, D['y_lid0'], 'lid', 'y')
    s.rect(-bx, zb, bx, zt)
    body = extrude(c, s, P['lid_t'], NEW, name='Upper').bodies.item(0)
    s = S(c, D['y_lid0'], 'lid holes', 'y')
    for (x, z) in D['body_posts']:
        s.circle(x, z, P['m3_clear'])
    # servo cable slots (servo connectors sit low on its ±X faces, back end)
    s.rect(-12.0, zb + P['tower_floor'] + 1.0, 4.0, zb + P['tower_floor'] + 17.0)
    s.rect(36.0, zb + P['tower_floor'] + 1.0, 44.0, zb + P['tower_floor'] + 17.0)
    extrude(c, s, P['lid_t'], CUT, body, 'lid holes + cable slots')
    # bosses on the back face for the palm-upstand screws, insert holes from the front
    uz = D['ups_screw_z']
    s = S(c, D['y_lid0'], 'upstand bosses', 'y')
    for x in P['upstand_screw_x']:
        s.circle(x, uz, 8.0)
    extrude(c, s, -4.0, JOIN, body, 'upstand bosses')
    s = S(c, P['lid_front'], 'upstand inserts', 'y')
    for x in P['upstand_screw_x']:
        s.circle(x, uz, P['insert_d'])
    extrude(c, s, -(P['lid_t'] + 4.0), CUT, body, 'upstand insert holes')
    # tower box in front of the lid (open top; floor; walls)
    tx, ty0, ty1, tw = P['tower_x'], D['tower_y0'], P['tower_y1'], P['tower_wall']
    th = D['z_servo_top'] - zb
    s = S(c, zb, 'tower outer')
    s.rect(-tx, ty0, tx, ty1)
    extrude(c, s, th, JOIN, body, 'tower')
    s = S(c, zb + P['tower_floor'], 'tower inner')
    s.rect(-tx + tw, ty0, tx - tw, ty1 - tw)
    extrude(c, s, th - P['tower_floor'], CUT, body, 'tower hollow')
    tp = P['tower_post'] / 2
    s = S(c, zb + P['tower_floor'], 'tower posts')
    for (x, y) in D['tower_posts']:
        s.rect(x - tp, y - tp, x + tp, y + tp)
    extrude(c, s, th - P['tower_floor'], JOIN, body, 'tower posts')
    s = S(c, D['z_servo_top'], 'tower inserts')
    for (x, y) in D['tower_posts']:
        s.circle(x, y, P['insert_d'])
    extrude(c, s, -P['insert_depth'], CUT, body, 'tower insert holes')
    # idler-horn clearance through the tower floor
    s = S(c, zb, 'idler hole')
    s.circle(D['spline_x'], D['py'], P['idler_hole_d'])
    extrude(c, s, P['tower_floor'], CUT, body, 'idler horn clearance')
    return c


def build_palm(root, P, D):
    c = new_comp(root, GEN_NAMES[2])
    px, y0, y1 = P['palm_x'], P['lid_front'], P['tower_y1']
    s = S(c, D['z_palm0'], 'palm')
    s.rect(-px, y0, px, y1)
    body = extrude(c, s, P['palm_t'], NEW, name='Palm').bodies.item(0)
    # rear upstand: ties palm + cover to the lid plate
    s = S(c, D['z_palm1'], 'upstand')
    s.rect(-px, D['ups_y0'], px, D['ups_y1'])
    extrude(c, s, D['z_cov0'] - D['z_palm1'], JOIN, body, 'upstand')
    uz = D['ups_screw_z']
    s = S(c, D['ups_y1'], 'upstand screw holes', 'y')
    for x in P['upstand_screw_x']:
        s.circle(x, uz, P['m3_clear'])
    extrude(c, s, -P['upstand_t'], CUT, body, 'upstand screw holes')
    s = S(c, D['ups_y1'], 'upstand cbores', 'y')
    for x in P['upstand_screw_x']:
        s.circle(x, uz, P['m3_head_d'])
    extrude(c, s, -3.0, CUT, body, 'upstand counterbores')
    s = S(c, D['z_cov0'], 'cover inserts')
    for x in P['cover_screw_x']:
        s.circle(x, (D['ups_y0'] + D['ups_y1']) / 2, P['insert_d'])
    extrude(c, s, -P['insert_depth'], CUT, body, 'cover insert holes')
    # plate holes
    sx, py = D['spline_x'], D['py']
    s = S(c, D['z_palm0'], 'palm holes')
    s.circle(sx, py, P['palm_horn_hole'])
    for (x, y) in [(-sx, py), (sx + P['s'], py), (-sx - P['s'], py)]:
        s.circle(x, y, P['insert_d'])
    for (x, y) in D['tower_posts']:
        s.circle(x, y, P['m3_clear'])
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
    ex, ey = gx + sgn * D['arm_end_dx'], D['arm_end_y']
    s = S(c, D['z_arm0'], 'arm')
    s.slot(gx, gy, ex, ey, P['link_w'])
    extrude(c, s, D['z_link1'] - D['z_arm0'], JOIN, body, 'arm')
    if right:
        s = S(c, D['z_gear1'], 'boss')
        s.circle(gx, gy, P['boss_d'])
        extrude(c, s, D['z_cov1'] - D['z_gear1'], JOIN, body, 'boss')
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
    ix = sgn * (D['spline_x'] + D['arm_end_dx'])
    ox = ix + sgn * P['s']
    y = D['arm_end_y']
    s = S(c, D['z_car0'], 'carrier')
    s.slot(ix, y, ox, y, P['carrier_w'])
    body = extrude(c, s, P['link_t'], NEW, name=('Carrier R' if right else 'Carrier L')).bodies.item(0)
    xj_in = ix - sgn * D['e']
    xj_out = xj_in + sgn * P['jaw_t']
    yj0 = y + P['carrier_w'] / 2 + P['jaw_gap_y']
    yj1 = yj0 + P['jaw_len']
    s = S(c, D['z_car0'], 'tongue')
    s.rect(min(xj_in, ix + sgn * 5), y - P['carrier_w'] / 2, max(xj_in, ix + sgn * 5), yj0 + 0.5)
    extrude(c, s, P['link_t'], JOIN, body, 'tongue')
    s = S(c, D['z_palm1'], 'jaw')
    s.rect(min(xj_in, xj_out), yj0, max(xj_in, xj_out), yj1)
    extrude(c, s, D['z_car1'] - D['z_palm1'], JOIN, body, 'jaw')
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
    px, y0, y1 = P['palm_x'], P['lid_front'], P['tower_y1']
    s = S(c, D['z_cov0'], 'cover')
    s.rect(-px, y0, px, y1)
    body = extrude(c, s, P['cover_t'], NEW, name='Cover').bodies.item(0)
    sx, py = D['spline_x'], D['py']
    s = S(c, D['z_cov0'], 'cover holes')
    s.circle(sx, py, P['boss_d'] + 2 * P['boss_clear'])
    for (x, y) in [(-sx, py), (sx + P['s'], py), (-sx - P['s'], py)]:
        s.circle(x, y, P['m3_clear'])
    for x in P['cover_screw_x']:
        s.circle(x, (D['ups_y0'] + D['ups_y1']) / 2, P['m3_clear'])
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
    # T-CAN485 standing on edge (plane XZ)
    s = S(c, D['pcb_y'], 'pcb', 'y')
    s.rect(D['pcb_x0'], D['pcb_z0'], -D['pcb_x0'], D['pcb_z0'] + P['pcb_w'])
    extrude(c, s, P['pcb_t'], NEW, name='REF T-CAN485 PCB 80x35 (standing)')
    s = S(c, D['pcb_y'] + P['pcb_t'], 'components', 'y')
    s.rect(D['pcb_x0'] - P['pcb_conn_over'], D['pcb_z0'], -D['pcb_x0'] + P['pcb_conn_over'],
           D['pcb_z0'] + P['pcb_w'])
    extrude(c, s, P['pcb_comp_h'], NEW, name='REF components / terminals / USB envelope')
    # UR flange disc behind the back face
    s = S(c, D['y_back'], 'ur flange', 'y')
    s.circle(0, D['z_fl'], 63.0)
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
        msg = ('gripper v2 built: opening closed=%.1f open=%.1f pose=%.1f mm; flange face y=%.1f, '
               'flange axis z=%.2f; jaw centre y=%.1f -> TCP z_t=%.1f mm; body z %.1f..%.1f; total y extent %.1f'
               % (D['opening_closed'], D['opening_open'], D['opening_pose'], D['y_back'], D['z_fl'],
                  D['jaw_y_mid'], D['jaw_y_mid'] - D['y_back'], P['z_bot'], D['z_top'],
                  (D['arm_end_y'] + P['carrier_w'] / 2 + P['jaw_gap_y'] + P['jaw_len']) - D['y_back']))
        print(msg)
        return msg
    except Exception:
        print('FAILED:\n' + traceback.format_exc())
        raise
