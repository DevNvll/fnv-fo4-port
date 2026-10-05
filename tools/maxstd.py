"""Evaluator for the standard controllers of a 3ds Max 2014 scene file.

The module reads the objects of maxscene.Scene and computes the same values
as Control::GetValue of the 3ds Max SDK. It uses the conventions of maxmath.py:
row vectors, a (4,3) matrix, mul(a, b) = "a * b" of the SDK, Quat (x, y, z, w).
A time is in ticks (4800 for each second).

Classes: Bezier Float, Bezier Point3, Bezier Color, Bezier Position, Bezier
Scale, Linear Rotation, Position XYZ, Euler XYZ, ScaleXYZ,
Position/Rotation/Scale, Float List, Point3 List, Position List, Rotation
List, Scale List, Position Constraint, Orientation Constraint, Link
Constraint, LinkTimeControl, ParamBlock2, ParamBlock, Noise Float, Noise
Position, Noise Point3, Noise Rotation, Noise Scale.

A function raises NotImplementedError for a class or a data layout that the
module cannot evaluate. The set StdEval.unverified gets one name for each code
path that ran and that no file data could prove (see maxstd-notes.md).
"""
import bisect
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maxmath as mm
from maxchunks import parse, load_stream

# (classid a, classid b, superclass) -> short name
CLASSES = {
    (0x2007, 0, 0x9003): 'bezfloat',
    (0x200a, 0, 0x9005): 'bezpoint3',
    (0x2011, 0, 0x9005): 'bezpoint3',       # Bezier Color: same layout
    (0x2008, 0, 0x900b): 'bezpos',
    (0x2010, 0, 0x900d): 'bezscale',
    (0x2003, 0, 0x900c): 'linrot',
    (0x118f7e02, 0xffee238a, 0x900b): 'posxyz',
    (0x2012, 0, 0x900c): 'euler',
    (0x118f7c01, 0xfeee238b, 0x900d): 'scalexyz',
    (0x2005, 0, 0x9008): 'prs',
    (0x4b4b1000, 0, 0x9003): 'floatlist',
    (0x4b4b1001, 0, 0x9005): 'point3list',
    (0x4b4b1002, 0, 0x900b): 'poslist',
    (0x4b4b1003, 0, 0x900c): 'rotlist',
    (0x4b4b1004, 0, 0x900d): 'scalelist',
    (0x2019, 0, 0x900b): 'poscon',
    (0x2020, 0, 0x900c): 'oricon',
    (0x873fe764, 0xaabe8601, 0x9008): 'link',
    (0x5d084b4b, 0x1b1d318a, 0x9003): 'linktime',
    (0x82, 0, 0x82): 'pblock2',
    (0x8, 0, 0x8): 'pblock',
    (0x87a6df24, 0, 0x9003): 'noisefloat',
    (0x87a6df25, 0, 0x900b): 'noisepos',
    (0x87a6df26, 0, 0x9005): 'noisepoint3',
    (0x87a6df27, 0, 0x900c): 'noiserot',
    (0x87a6df28, 0, 0x900d): 'noisescale',
}

FLOAT_KINDS = ('bezfloat', 'floatlist', 'linktime', 'noisefloat')
POINT3_KINDS = ('bezpoint3', 'bezpos', 'posxyz', 'point3list', 'poslist', 'poscon', 'noisepos', 'noisepoint3')
ROT_KINDS = ('linrot', 'euler', 'rotlist', 'oricon', 'noiserot')
SCALE_KINDS = ('bezscale', 'scalexyz', 'scalelist', 'noisescale')
NOISE_KINDS = ('noisefloat', 'noisepos', 'noisepoint3', 'noiserot', 'noisescale')
MATRIX_KINDS = ('prs', 'link')

# Bezier key flags (istdplug.h)
BEZKEY_SMOOTH, BEZKEY_LINEAR, BEZKEY_STEP, BEZKEY_FAST, BEZKEY_SLOW, BEZKEY_USER, BEZKEY_FLAT = range(7)
TAN_NAMES = ('smooth', 'linear', 'step', 'fast', 'slow', 'custom', 'auto')
TFLAG_RANGE_UNLOCKED = 1 << 1
# Out-of-range types (control.h)
ORT_CONSTANT, ORT_CYCLE, ORT_LOOP, ORT_OSCILLATE, ORT_LINEAR, ORT_IDENTITY, ORT_RELATIVE_REPEAT = range(1, 8)
ORT_NAMES = {1: 'constant', 2: 'cycle', 3: 'loop', 4: 'oscillate', 5: 'linear', 6: 'identity', 7: 'relative repeat'}
# Inheritance flags (control.h). In the file a set bit means "do not inherit".
INHERIT_POS = 0x007
INHERIT_ROT = 0x038
INHERIT_SCL = 0x1c0

EULER_ORDERINGS = ((0, 1, 2), (0, 2, 1), (1, 2, 0), (1, 0, 2), (2, 0, 1), (2, 1, 0), (0, 1, 0), (1, 2, 1), (2, 0, 2))
EULER_NAMES = ('XYZ', 'XZY', 'YZX', 'YXZ', 'ZXY', 'ZYX', 'XYX', 'YZY', 'ZXZ')

TIME_NEG_INF = -2 ** 31
TIME_POS_INF = 2 ** 31 - 1
DEFAULT_LEN = 0.3333            # the value that the file has for a tangent length that the user did not change

# ParamBlock2 (iparamb2.h, paramtype.h)
P_ANIMATABLE = 0x1
P_NO_REF = 0x100
P_OWNERS_REF = 0x200
P_SUBTEX = 0x800
TYPE_TAB = 0x800
PB2_REF_TYPES = (14, 15, 17, 18, 21)                    # mtl, texmap, inode, reftarg, pblock2
PB2_FLOAT_TYPES = (0, 5, 6, 7, 11)                      # float, angle, percent fraction, world, color channel
PB2_INT_TYPES = (1, 4, 12, 13, 19)                      # int, bool, time value, radio button index, index
PB2_POINT3_TYPES = (2, 3, 10)                           # rgba, point3, hsv
PB2_TYPE_NAMES = {0: 'float', 1: 'int', 2: 'rgba', 3: 'point3', 4: 'bool', 5: 'angle', 6: 'pcnt_frac', 7: 'world',
                  8: 'string', 9: 'filename', 10: 'hsv', 11: 'color_channel', 12: 'timevalue', 13: 'radiobtn_index',
                  14: 'mtl', 15: 'texmap', 16: 'bitmap', 17: 'inode', 18: 'reftarg', 19: 'index', 20: 'matrix3',
                  21: 'pblock2', 22: 'point4', 23: 'frgba', 30: 'point2'}


# ----------------------------------------------------------------------------
# Math helpers that maxmath.py does not have

def quat_sdk_mul(a, b):
    """Quat::operator*. The files prove that it is the Hamilton product a (x) b:
    MakeMatrix(a * b) == MakeMatrix(a) * MakeMatrix(b) (a first, then b)."""
    return mm.quat_hamilton(a, b)


def quat_sdk_div(p, q):
    """Quat::operator/: r = p / q is the quaternion with q * r == p."""
    return mm.quat_hamilton(mm.quat_inv(q), p)


def quat_closest(q, to):
    """Quat::MakeClosest."""
    q = np.asarray(q, float)
    return -q if float(np.dot(q, to)) < 0 else q


IDENT_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def decomp_affine(m):
    """decomp_affine: linear part = S * R * f. S is the symmetric stretch (Inverse(U) K U),
    R is the rotation of the quaternion q, f is +1 or -1."""
    w, sig, vt = np.linalg.svd(m[:3])
    rot = w @ vt
    stretch = (w * sig) @ w.T
    f = 1.0
    if np.linalg.det(rot) < 0:
        f = -1.0
        rot = -rot
    return {'t': m[3].copy(), 'R': rot, 'S': stretch, 'f': f}


def comp_affine(parts):
    r = np.empty((4, 3))
    r[:3] = parts['S'] @ parts['R'] * parts['f']
    r[3] = parts['t']
    return r


def apply_scaling(m, s, q=None):
    """ApplyScaling(m, ScaleValue(s, q)): m = Inverse(U) * ScaleMatrix(s) * U * m, U = q.MakeMatrix."""
    if q is None or abs(abs(q[3]) - 1.0) < 1e-12:
        return mm.pre_scale(m, s)
    u = mm.quat_to_mat(q)[:3]
    r = m.copy()
    r[:3] = u.T @ np.diag(np.asarray(s, float)) @ u @ m[:3]
    return r


def cycle_time(start, end, t):
    """CycleTime of control.h (the C operator % truncates toward zero)."""
    dur = (end - start + 1) - 1
    if dur <= 0:
        return t
    res = int(math.fmod(t - start, dur))
    return end + res if t < start else start + res


def num_cycles(start, end, t):
    """NumCycles of control.h."""
    dur = (end - start + 1) - 1
    if dur <= 0:
        return 1
    if t < start:
        return int(abs(t - start) // dur) + 1
    if t > end:
        return int(abs(t - end) // dur) + 1
    return 0


def _bez_scalar(t, t0, v0, ot, ol, t1, v1, it, il):
    """One Bezier segment. Control points: (t0, v0), (t0 + ol*dt, v0 + ot*ol*dt),
    (t1 - il*dt, v1 + it*il*dt), (t1, v1). ot and it are slopes for each tick; it points back in time."""
    dt = float(t1 - t0)
    std_o = ol <= 0.0 or abs(ol - DEFAULT_LEN) < 1e-6 or abs(ol - 1.0 / 3.0) < 1e-6
    std_i = il <= 0.0 or abs(il - DEFAULT_LEN) < 1e-6 or abs(il - 1.0 / 3.0) < 1e-6
    if std_o and std_i:
        u = (t - t0) / dt
        p1 = v0 + ot * dt / 3.0
        p2 = v1 + it * dt / 3.0
    else:
        if std_o:
            ol = 1.0 / 3.0
        if std_i:
            il = 1.0 / 3.0
        x1 = t0 + ol * dt
        x2 = t1 - il * dt
        lo, hi = 0.0, 1.0
        for _ in range(60):
            u = 0.5 * (lo + hi)
            w = 1.0 - u
            x = w * w * w * t0 + 3.0 * w * w * u * x1 + 3.0 * w * u * u * x2 + u * u * u * t1
            if x < t:
                lo = u
            else:
                hi = u
        u = 0.5 * (lo + hi)
        p1 = v0 + ot * ol * dt
        p2 = v1 + it * il * dt
    w = 1.0 - u
    return w * w * w * v0 + 3.0 * w * w * u * p1 + 3.0 * w * u * u * p2 + u * u * u * v1


_F = np.float32
_PERLIN = None


def _perlin_tables():
    """The tables of perlin.cpp (init): srand(0) and rand() of the Microsoft C library."""
    global _PERLIN
    if _PERLIN is None:
        size = 0x100
        hold = [0]

        def rnd():
            hold[0] = (hold[0] * 214013 + 2531011) & 0xffffffff
            return (hold[0] >> 16) & 0x7fff
        p = [0] * (size + size + 2)
        g1 = [_F(0)] * (size + size + 2)
        for i in range(size):
            p[i] = i
            g1[i] = _F((rnd() % (size + size)) - size) / _F(size)
            for _ in range(5):          # g2[i][0..1] and g3[i][0..2]: noise1 does not use them
                rnd()
        i = size
        while True:
            i -= 1
            if i == 0:
                break
            k = p[i]
            j = rnd() % size
            p[i] = p[j]
            p[j] = k
        for i in range(size + 2):
            p[size + i] = p[i]
            g1[size + i] = g1[i]
        _PERLIN = (p, g1)
    return _PERLIN


def perlin_perm(v):
    """Perm of perlin.cpp."""
    return _perlin_tables()[0][v & 0xff]


def perlin_noise1(arg):
    """noise1 of perlin.cpp with the float (32 bit) arithmetic of the C code."""
    p, g1 = _perlin_tables()
    t = _F(arg) + _F(0x1000)
    it = int(t)
    bx0 = it & 0xff
    bx1 = (bx0 + 1) & 0xff
    rx0 = t - _F(it)
    rx1 = _F(float(rx0) - 1.0)
    sx = _F(float(rx0) * float(rx0) * (3.0 - 2.0 * float(rx0)))
    u = rx0 * g1[p[bx0]]
    v = rx1 * g1[p[bx1]]
    return u + sx * (v - u)


def perlin_fbm1(point, h, lacunarity, octaves):
    """fBm1 of perlin.cpp."""
    point = float(point)
    value = 0.0
    freq = 1.0
    n = int(octaves)
    for _ in range(n):
        value += float(perlin_noise1(_F(point))) * math.pow(freq, -h)
        freq *= lacunarity
        point *= lacunarity
    rem = octaves - n
    if rem:
        value += rem * float(perlin_noise1(_F(point))) * math.pow(freq, -h)
    return value


class PBParam:
    """One parameter of a ParamBlock2. entries is a list of (kind, value, ref):
    kind 'val' (constant), 'ctl' (a controller or another reference in a reference slot of an
    animatable parameter), 'ref' (reference that the block owns), 'idx' (scene index that the
    block stores with no reference), 'str', 'bitmap', 'none'."""
    __slots__ = ('id', 'type', 'flags', 'is_tab', 'entries')

    def __init__(self, pid, typ, flags, is_tab, entries):
        self.id = pid
        self.type = typ
        self.flags = flags
        self.is_tab = is_tab
        self.entries = entries

    @property
    def type_name(self):
        return PB2_TYPE_NAMES.get(self.type & 0x7ff, '?%d' % (self.type & 0x7ff)) + ('_tab' if self.is_tab else '')


def time_config(folder):
    """Read the time configuration from the Config stream of an extracted scene folder.
    Chunk 0x20b0: 0x0010 frames for each second, 0x0050 start, 0x0060 end, 0x0070 time slider (ticks)."""
    top = parse(load_stream(os.path.join(folder, 'Config')))
    tc = None
    for c in top:
        if c.id == 0x20b0 and c.children is not None:
            tc = c
            break
    if tc is None:
        raise NotImplementedError('Config stream has no time configuration chunk 0x20b0')

    def get(cid):
        c = tc.find(cid)
        if c is None or c.data is None or len(c.data) < 4:
            raise NotImplementedError('time configuration has no chunk 0x%04x' % cid)
        return struct.unpack_from('<i', c.data)[0]
    fps = get(0x0010)
    return {'fps': fps, 'ticks_per_frame': mm.TICKS_PER_SEC // fps, 'start': get(0x0050), 'end': get(0x0060),
            'slider': get(0x0070)}


def anim_range(scene):
    """(start_ticks, end_ticks, ticks_per_frame) of the animation range of a scene.
    scene is the path of the extracted folder, or a Scene object that has the attribute `folder`
    (maxscene.Scene does not keep its folder: pass the path, or set scene.folder)."""
    folder = scene if isinstance(scene, (str, os.PathLike)) else getattr(scene, 'folder', None)
    if folder is None:
        raise ValueError('anim_range needs the scene folder: give the path, or set scene.folder')
    tc = time_config(folder)
    return tc['start'], tc['end'], tc['ticks_per_frame']


class StdEval:
    def __init__(self, scene, node_tm, strict=False):
        """node_tm(node_index, t) -> (4,3) world matrix of a scene node at time t (ticks).
        strict=True raises NotImplementedError for each code path that no file data could prove."""
        self.scene = scene
        self.node_tm = node_tm
        self.strict = strict
        self.unverified = set()
        # noise_off = True: a noise controller gives its zero value (0, identity rotation, scale 1)
        self.noise_off = False
        # NoiseAtTime: v = t * factor * frequency. 0.005 until 3ds Max 2016; the SDK source of 2023 has 0.0055
        # (comment of 2015-11-25 in noizctrl.cpp). The scenes are from 3ds Max 2014.
        self.noise_time_factor = 0.005
        self._kind = {}
        self._keys = {}
        self._pb = {}
        self._cache = {}
        self._users = None

    # ------------------------------------------------------------------ basics

    def kind(self, idx):
        k = self._kind.get(idx, False)
        if k is False:
            k = None
            if 0 <= idx < len(self.scene.objs):
                cls = self.scene.objs[idx].cls
                if cls:
                    k = CLASSES.get((cls['classid'][0], cls['classid'][1], cls['super']))
            self._kind[idx] = k
        return k

    def handles(self, idx):
        return self.kind(idx) is not None

    def class_name(self, idx):
        if 0 <= idx < len(self.scene.objs):
            return self.scene.objs[idx].cname
        return '<no object %d>' % idx

    def _need(self, idx, kinds, what):
        k = self.kind(idx)
        if k is None:
            raise NotImplementedError('%s (object %d): class not covered by maxstd' % (self.class_name(idx), idx))
        if k not in kinds:
            raise TypeError('%s (object %d) is not a %s controller' % (self.class_name(idx), idx, what))
        return k

    def _flag(self, name, idx=None):
        """Record a code path that the file data could not prove."""
        if self.strict:
            raise NotImplementedError('%s: not verified%s' % (name, '' if idx is None else ' (%s, object %d)' % (self.class_name(idx), idx)))
        self.unverified.add(name)

    def _obj(self, idx):
        return self.scene.objs[idx]

    def _refs(self, idx):
        return self.scene.objs[idx].refs or []

    def _ref(self, idx, i):
        r = self._refs(idx)
        return r[i] if i < len(r) else -1

    def users(self, idx):
        """Scene indices of the objects that have a reference to object idx."""
        if self._users is None:
            u = {}
            for o in self.scene.objs:
                for r in (o.refs or []):
                    if r >= 0:
                        u.setdefault(r, []).append(o.index)
            self._users = u
        return self._users.get(idx, [])

    # ------------------------------------------------------- keyframe controllers

    def _body(self, idx):
        """The chunk that holds the data of a keyframe controller (0x7127 for float and Point3)."""
        o = self._obj(idx)
        c = o.chunk.find(0x7127)
        return c if c is not None else o.chunk

    def _ort(self, idx):
        """(before, after) out-of-range types. Control::Save writes them into chunk 0x8499
        (0x3000 before, 0x3001 after). No covered controller of the scenes has this chunk."""
        o = self._obj(idx)
        before = after = ORT_CONSTANT
        for holder in (o.chunk, self._body(idx)):
            c = holder.find(0x8499)
            if c is None:
                continue
            if c.children is None:
                raise NotImplementedError('%s (object %d): chunk 0x8499 is not a container' % (o.cname, idx))
            for cc in c.children:
                if cc.id == 0x3000 and cc.data is not None and len(cc.data) >= 4:
                    before = struct.unpack_from('<i', cc.data)[0]
                elif cc.id == 0x3001 and cc.data is not None and len(cc.data) >= 4:
                    after = struct.unpack_from('<i', cc.data)[0]
        return before, after

    def _keydata(self, idx):
        """Parse a keyframe controller: dict with static value, key arrays, range and flags."""
        d = self._keys.get(idx)
        if d is not None:
            return d
        k = self.kind(idx)
        o = self._obj(idx)
        body = self._body(idx)
        static_id, key_id, rec = {
            'bezfloat': (0x2501, 0x2525, 28),
            'bezpoint3': (0x2501, None, 0),
            'bezpos': (0x2503, None, 0),
            'linrot': (0x2504, None, 0),
            'bezscale': (0x2505, 0x2528, 148),
        }[k]
        sc = body.find(static_id)
        if sc is None or sc.data is None:
            raise NotImplementedError('%s (object %d): no static value chunk 0x%04x' % (o.cname, idx, static_id))
        n = len(sc.data) // 4
        static = np.array(struct.unpack('<%df' % n, sc.data[:n * 4]), float)
        for c in body.children or []:
            if 0x2510 <= c.id <= 0x252f and c.id != key_id:
                raise NotImplementedError('%s (object %d): key chunk 0x%04x (%d bytes) has no decoder; the scenes have no such keys'
                                          % (o.cname, idx, c.id, len(c.data) if c.data is not None else -1))
        if any(r >= 0 for r in (o.refs or [])):
            raise NotImplementedError('%s (object %d): ease or multiplier curves are not covered' % (o.cname, idx))
        iv = body.find(0x2500)
        rng = body.find(0x3003)
        tf = body.find(0x3002)
        d = {'kind': k, 'static': static,
             'valid': struct.unpack('<ii', iv.data) if iv is not None and iv.data and len(iv.data) == 8 else None,
             'range': struct.unpack('<ii', rng.data) if rng is not None and rng.data and len(rng.data) == 8 else None,
             'tflags': struct.unpack('<I', tf.data)[0] if tf is not None and tf.data and len(tf.data) == 4 else 0,
             'ort': self._ort(idx), 'times': [], 'flags': []}
        kc = body.find(key_id) if key_id is not None else None
        if kc is not None and kc.data:
            if len(kc.data) % rec:
                raise NotImplementedError('%s (object %d): key chunk size %d is not a multiple of %d' % (o.cname, idx, len(kc.data), rec))
            nk = len(kc.data) // rec
            if k == 'bezfloat':
                raw = [struct.unpack_from('<iIfffff', kc.data, i * rec) for i in range(nk)]
                d['times'] = [r[0] for r in raw]
                d['flags'] = [r[1] for r in raw]
                d['val'] = [r[2] for r in raw]
                d['intan'] = [r[3] for r in raw]
                d['outtan'] = [r[4] for r in raw]
                d['inlen'] = [r[5] for r in raw]
                d['outlen'] = [r[6] for r in raw]
            else:   # bezscale: time, flags, 5 ScaleValue (Point3 + Quat): val, intan, outtan, inLength, outLength
                raw = [(struct.unpack_from('<iI', kc.data, i * rec), struct.unpack_from('<35f', kc.data, i * rec + 8)) for i in range(nk)]
                d['times'] = [r[0][0] for r in raw]
                d['flags'] = [r[0][1] for r in raw]
                d['val'] = [r[1][0:3] for r in raw]
                d['valq'] = [np.array(r[1][3:7], float) for r in raw]
                d['intan'] = [r[1][7:10] for r in raw]
                d['outtan'] = [r[1][14:17] for r in raw]
                d['inlen'] = [r[1][21:24] for r in raw]
                d['outlen'] = [r[1][28:31] for r in raw]
            if d['times'] != sorted(d['times']):
                raise NotImplementedError('%s (object %d): key times are not sorted' % (o.cname, idx))
        self._keys[idx] = d
        return d

    def _seg(self, d, i, t, comp):
        """Value of component comp on the segment between key i and key i+1."""
        if comp is None:
            v0, v1 = d['val'][i], d['val'][i + 1]
            ot, it = d['outtan'][i], d['intan'][i + 1]
            ol, il = d['outlen'][i], d['inlen'][i + 1]
        else:
            v0, v1 = d['val'][i][comp], d['val'][i + 1][comp]
            ot, it = d['outtan'][i][comp], d['intan'][i + 1][comp]
            ol, il = d['outlen'][i][comp], d['inlen'][i + 1][comp]
        t0, t1 = d['times'][i], d['times'][i + 1]
        otype = (d['flags'][i] >> 10) & 7
        itype = (d['flags'][i + 1] >> 7) & 7
        if otype != BEZKEY_USER or itype != BEZKEY_USER:
            # The scenes have custom tangents only. Rules for the other types:
            # step holds the value of the first key, linear points at the other key,
            # smooth, fast, slow and auto use the tangent that the file has for the key.
            for ty in (otype, itype):
                if ty != BEZKEY_USER:
                    self._flag('tangent:' + TAN_NAMES[ty] if ty < 7 else 'tangent:%d' % ty)
            if otype == BEZKEY_STEP or itype == BEZKEY_STEP:
                return v0
            dt = float(t1 - t0)
            if otype == BEZKEY_LINEAR:
                ot = (v1 - v0) / dt
            if itype == BEZKEY_LINEAR:
                it = (v0 - v1) / dt
            if otype != BEZKEY_USER:
                ol = DEFAULT_LEN
            if itype != BEZKEY_USER:
                il = DEFAULT_LEN
        return _bez_scalar(t, t0, v0, ot, ol, t1, v1, it, il)

    def _local(self, d, t, comp):
        """Value at a time inside the key range (GetValueLocalTime)."""
        times = d['times']
        if t <= times[0]:
            return d['val'][0] if comp is None else d['val'][0][comp]
        if t >= times[-1]:
            return d['val'][-1] if comp is None else d['val'][-1][comp]
        i = bisect.bisect_right(times, t) - 1
        if times[i] == t:
            return d['val'][i] if comp is None else d['val'][i][comp]
        return self._seg(d, i, t, comp)

    def _keyed(self, d, t, comp=None):
        """Value of a keyframe controller with the out-of-range types (StdControl::GetValue)."""
        times = d['times']
        if not times:
            return d['static'][0] if comp is None else d['static'][comp]
        if len(times) == 1:
            return d['val'][0] if comp is None else d['val'][0][comp]
        start, end = times[0], times[-1]
        if d['tflags'] & TFLAG_RANGE_UNLOCKED and d['range'] is not None:
            self._flag('range:unlocked')
            start, end = d['range']
        if start <= t <= end:
            return self._local(d, t, comp)
        ort = d['ort'][0] if t < start else d['ort'][1]
        if ort == ORT_CONSTANT:
            return self._local(d, start if t < start else end, comp)
        self._flag('ort:' + ORT_NAMES.get(ort, str(ort)))
        if ort in (ORT_CYCLE, ORT_LOOP):
            return self._local(d, cycle_time(start, end, t), comp)
        if ort == ORT_OSCILLATE:
            tp = cycle_time(start, end, t)
            if num_cycles(start, end, t) & 1:
                tp = end - (tp - start)
            return self._local(d, tp, comp)
        if ort == ORT_LINEAR:
            if t < start:
                v0 = self._local(d, start, comp)
                v1 = self._local(d, start + 1, comp)
                return v0 + (v1 - v0) * float(t - start)
            v0 = self._local(d, end - 1, comp)
            v1 = self._local(d, end, comp)
            return v1 + (v1 - v0) * float(t - end)
        if ort == ORT_IDENTITY:
            if t < start:
                return self._local(d, start, comp) + float(t - start)
            return self._local(d, end, comp) + float(t - end)
        if ort == ORT_RELATIVE_REPEAT:
            v0 = self._local(d, start, comp)
            v1 = self._local(d, end, comp)
            v2 = self._local(d, cycle_time(start, end, t), comp)
            delta = (v0 - v1) if t < start else (v1 - v0)
            return v2 + delta * float(num_cycles(start, end, t))
        raise NotImplementedError('out-of-range type %d' % ort)

    def cached_value(self, idx):
        """(validity interval, value) that the file has for a keyframe controller: the last value
        that 3ds Max computed before the save. The interval is (t, t) for an animated controller."""
        d = self._keydata(idx)
        return d['valid'], d['static']

    # ------------------------------------------------------------------ float

    def float_value(self, idx, t):
        k = self._need(idx, FLOAT_KINDS, 'float')
        if k == 'bezfloat':
            return float(self._keyed(self._keydata(idx), t))
        if k == 'noisefloat':
            return float(self._noise(idx, t)[0])
        if k == 'floatlist':
            conts, weights, avg = self._list(idx, t)
            v = 0.0
            for c, w in zip(conts, weights):
                if w != 0.0 and c >= 0:
                    v += self.float_value(c, t) * self._avg(w, avg)
            return v
        # LinkTimeControl::GetValueLocalTime: index of the link that is active at t
        owner = [u for u in self.users(idx) if self.kind(u) == 'link']
        if not owner:
            return 0.0
        nodes, times = self._link_params(owner[0])
        if not nodes:
            return 0.0
        for i in range(1, len(nodes)):
            if times[i] > t:
                return float(i - 1)
        return float(len(nodes) - 1)

    # ----------------------------------------------------------------- lists

    def _list(self, idx, t):
        """(sub-controller indices, weights at t, total weight at time 0 or None)."""
        info = self._cache.get(('list', idx))
        if info is None:
            o = self._obj(idx)
            cc = o.chunk.find(0x1010)
            count = struct.unpack('<i', cc.data)[0] if cc is not None else 0
            refs = o.refs or []
            # references: the sub-controllers, the "Available" slot, the clipboard, the ParamBlock2
            conts = [refs[i] if i < len(refs) else -1 for i in range(count)]
            pb = refs[count + 2] if len(refs) > count + 2 else -1
            if count and (pb < 0 or self.kind(pb) != 'pblock2'):
                raise NotImplementedError('%s (object %d): no weight ParamBlock2' % (o.cname, idx))
            info = (conts, pb)
            self._cache[('list', idx)] = info
        conts, pb = info
        if not conts:
            return [], [], None
        n = self.pblock2_count(pb, 0)
        if n != len(conts):
            # ListCtrlPostLoad fills the missing weights with 1.0
            self._flag('list:weight count differs from the list count', idx)
        weights = [self.pblock2_float(pb, 0, t, i) if i < n else 1.0 for i in range(len(conts))]
        avg = None
        if self.pblock2(pb).get(1, 0):
            self._flag('list:average weights', idx)
            avg = sum(self.pblock2_float(pb, 0, 0, i) if i < n else 1.0 for i in range(len(conts)))
        return conts, weights, avg

    @staticmethod
    def _avg(w, total):
        """ListControl::AverageWeight."""
        if total is None:
            return w
        return w / total if total != 0.0 else 0.0

    # --------------------------------------------------------- position, Point3

    def point3_value(self, idx, t):
        """Absolute value (CTRL_ABSOLUTE) of a position or Point3 controller."""
        k = self._need(idx, POINT3_KINDS, 'position or Point3')
        if k in ('bezpoint3', 'bezpos'):
            d = self._keydata(idx)
            return d['static'][:3].copy()        # _keydata raises when the controller has keys
        if k in ('noisepos', 'noisepoint3'):
            return self._noise(idx, t)
        if k == 'posxyz':
            return np.array([self.float_value(r, t) if r >= 0 else 0.0 for r in (self._ref(idx, i) for i in range(3))])
        if k == 'point3list':
            conts, weights, avg = self._list(idx, t)
            v = np.zeros(3)
            for c, w in zip(conts, weights):
                if w != 0.0 and c >= 0:
                    v = v + self.point3_value(c, t) * self._avg(w, avg)
            return v
        if k == 'poslist':
            conts, weights, avg = self._list(idx, t)
            tm = mm.ident()
            for c, w in zip(conts, weights):
                if w != 0.0:
                    prev = tm[3].copy()
                    if c >= 0:
                        tm = self.apply(c, t, tm)
                    tm[3] = prev + (tm[3] - prev) * self._avg(w, avg)
            return tm[3].copy()
        return self._poscon(idx, t)

    def _poscon(self, idx, t):
        """PosConstPosition::Update."""
        o = self._obj(idx)
        pb = self._ref(idx, 0)
        if pb < 0:
            raise NotImplementedError('Position Constraint (object %d): no ParamBlock2' % idx)
        p = self.pblock2(pb)
        nodes = p.get(1, [])
        ct = len(nodes)
        base_world = np.array(struct.unpack('<3f', o.chunk.find(0x1002).data), float)
        initial = np.array(struct.unpack('<3f', o.chunk.find(0x1003).data), float)
        old_ct = struct.unpack('<i', o.chunk.find(0x1004).data)[0]
        cur = np.zeros(3)
        total = 0.0
        nw = self.pblock2_count(pb, 0)
        for i, n in enumerate(nodes):
            if n is None or n < 0:
                continue
            w = self.pblock2_float(pb, 0, t, i) if i < nw else 0.0
            total += w
            cur = cur + w * np.asarray(self.node_tm(n, t), float)[3]
        if total > 0.0:
            cur = cur / total
            if p.get(2, 0):
                if old_ct != ct:
                    raise NotImplementedError('Position Constraint (object %d): the target count changed after the '
                                              'initial position was stored; 3ds Max sets it at the first evaluation' % idx)
                cur = cur + (base_world - initial)
        else:
            cur = base_world
        return cur

    # --------------------------------------------------------------- rotation

    def _euler(self, idx, t):
        """EulerRotation::GetValue: the matrix of the rotation, (4,3)."""
        o = self._obj(idx)
        oc = o.chunk.find(0x1003)
        raw = struct.unpack('<i', oc.data)[0] if oc is not None else 0
        refs = [self._ref(idx, i) for i in range(3)]
        if raw // 100 > 0:
            order = raw % 100
        else:
            order = raw
            if 0 < order < 6:
                # file of an old version: reference i is the i-th angle (RemapRefOnLoad)
                self._flag('euler:old reference order', idx)
                slots = [-1, -1, -1]
                for i in range(3):
                    slots[EULER_ORDERINGS[order][i]] = refs[i]
                refs = slots
        if not 0 <= order < 9:
            raise NotImplementedError('Euler XYZ (object %d): axis order %d' % (idx, order))
        if order != 0:
            self._flag('euler:order ' + EULER_NAMES[order], idx)
        ang = [self.float_value(r, t) if r >= 0 else 0.0 for r in refs]       # the X, Y and Z angle
        axes = EULER_ORDERINGS[order]
        cur = [ang[axes[i]] for i in range(3)] if order < 6 else ang
        return mm.euler_to_mat(cur, axes)

    def _oricon(self, idx, t):
        """OrientConstRotation::Update: (curRot, number of targets, local flag)."""
        o = self._obj(idx)
        pb = self._ref(idx, 0)
        if pb < 0:
            raise NotImplementedError('Orientation Constraint (object %d): no ParamBlock2' % idx)
        p = self.pblock2(pb)
        nodes = p.get(1, [])
        ct = len(nodes)
        relative = bool(p.get(2, 0))
        local = bool(p.get(3, 0))
        base_local = np.array(struct.unpack('<4f', o.chunk.find(0x1001).data), float)
        base_world = np.array(struct.unpack('<4f', o.chunk.find(0x1002).data), float)
        initial = np.array(struct.unpack('<4f', o.chunk.find(0x1003).data), float)
        old_ct = struct.unpack('<i', o.chunk.find(0x1004).data)[0]
        nw = self.pblock2_count(pb, 0)
        total = 0.0
        prev = IDENT_QUAT
        cur = IDENT_QUAT
        for i, n in enumerate(nodes):
            w = self.pblock2_float(pb, 0, t, i) if i < nw else 0.0
            if n is None or n < 0:
                tm = mm.ident()
            else:
                tm = np.asarray(self.node_tm(n, t), float)
                if local:
                    self._flag('orientation constraint:local mode', idx)
                    par = self.scene.objs[n].parent
                    if par is not None and 0 <= par < len(self.scene.objs) and self.scene.objs[par].cname == 'Node':
                        tm = mm.mul(tm, mm.inv(np.asarray(self.node_tm(par, t), float)))
            q = quat_closest(mm.quat_norm(mm.mat_to_quat(mm.no_trans(comp_affine_rot(tm)))), prev)
            if i == 0:
                cur = q
            else:
                sw = w / (total + w) if (total + w) != 0.0 else 0.0
                cur = mm.quat_norm(mm.quat_slerp(prev, q, sw))
            prev = cur
            total += w
        lcur = cur
        if total > 0.0:
            if relative:
                if old_ct != ct:
                    raise NotImplementedError('Orientation Constraint (object %d): the target count changed after the '
                                              'initial orientation was stored; 3ds Max sets it at the first evaluation' % idx)
                cur = quat_sdk_mul(base_local if local else base_world, quat_sdk_div(lcur, initial))
        else:
            cur = base_local
        cur = mm.quat_norm(quat_closest(cur, IDENT_QUAT))
        return cur, ct, local

    def quat_value(self, idx, t):
        """Absolute value (CTRL_ABSOLUTE) of a rotation controller: SDK Quat (x, y, z, w)."""
        k = self._need(idx, ROT_KINDS, 'rotation')
        if k == 'linrot':
            return self._keydata(idx)['static'][:4].copy()      # _keydata raises when the controller has keys
        if k == 'euler':
            return mm.mat_to_quat(self._euler(idx, t))
        if k == 'noiserot':
            return mm.mat_to_quat(self._noise_rot(idx, t))
        if k == 'rotlist':
            return mm.mat_to_quat(self._rotlist(idx, t, mm.ident()))
        return self._oricon(idx, t)[0]

    def _rotlist(self, idx, t, tm):
        """RotationListControl::GetValue."""
        conts, weights, avg = self._list(idx, t)
        if avg is None:
            for c, w in zip(conts, weights):
                if w != 0.0 and c >= 0:
                    local = mm.mul(self.apply(c, t, tm), mm.inv(tm))
                    q = quat_closest(mm.mat_to_quat(local), IDENT_QUAT)
                    q = mm.quat_norm(mm.quat_slerp(IDENT_QUAT, q, min(max(w, 0.0), 1.0)))
                    tm = mm.pre_rotate(tm, q)
            return tm
        init = tm.copy()
        qinit = mm.mat_to_quat(init)
        last = IDENT_QUAT
        for c, w in zip(conts, weights):
            if w != 0.0 and c >= 0:
                local = mm.mul(self.apply(c, t, tm), mm.inv(tm))
                q = quat_closest(mm.mat_to_quat(local), last)
                q = mm.quat_norm(mm.quat_slerp(last, q, min(max(w, 0.0), 1.0)))
                last = q
                r = mm.quat_to_mat(quat_sdk_mul(q, qinit))        # Matrix3::SetRotate removes the scale
                r[3] = tm[3]
                tm = r
        return tm

    # ------------------------------------------------------------------ scale

    def scale_value(self, idx, t, with_axis=False):
        """Absolute value of a scale controller: np.array(3). With with_axis=True the result is
        (scale, axis quaternion) of the ScaleValue."""
        k = self._need(idx, SCALE_KINDS, 'scale')
        q = IDENT_QUAT.copy()
        if k == 'bezscale':
            d = self._keydata(idx)
            if not d['times']:
                s = d['static'][:3].copy()
                q = d['static'][3:7].copy()
            else:
                s = np.array([self._keyed(d, t, c) for c in range(3)])
                times = d['times']
                if t <= times[0] or len(times) == 1:
                    q = d['valq'][0].copy()
                elif t >= times[-1]:
                    q = d['valq'][-1].copy()
                else:
                    i = bisect.bisect_right(times, t) - 1
                    q0, q1 = d['valq'][i], d['valq'][i + 1]
                    if np.abs(q0 - q1).max() < 1e-6:
                        q = q0.copy()
                    else:
                        self._flag('scale:interpolation of the axis quaternion', idx)
                        q = mm.quat_norm(mm.quat_slerp(q0, q1, (t - times[i]) / float(times[i + 1] - times[i])))
        elif k == 'scalexyz':
            s = np.array([self.float_value(r, t) if r >= 0 else 1.0 for r in (self._ref(idx, i) for i in range(3))])
        elif k == 'noisescale':
            s = 1.0 + self._noise(idx, t)
        else:
            conts, weights, avg = self._list(idx, t)
            s = np.ones(3)
            for c, w in zip(conts, weights):
                if w != 0.0 and c >= 0:
                    self._flag('scale list:entries', idx)
                    cs, cq = self.scale_value(c, t, True)
                    if abs(abs(cq[3]) - 1.0) > 1e-9:
                        raise NotImplementedError('Scale List (object %d): an entry has a scale axis quaternion' % idx)
                    s = s * (cs * self._avg(w, avg))        # ScaleValue + ScaleValue multiplies
        return (s, q) if with_axis else s

    # ------------------------------------------------------------------ apply

    def inherit_flags(self, idx):
        """Inheritance flags of a Position/Rotation/Scale controller (chunk 0x7230), or None.
        A set bit means that the node does NOT inherit that part: bits 0-2 position X, Y, Z,
        bits 3-5 rotation, bits 6-8 scale. 0 inherits all. 0x1c0 does not inherit the scale."""
        if self.kind(idx) != 'prs':
            return None
        o = self._obj(idx)
        a = o.chunk.find(0x7230)
        if a is None or a.data is None or len(a.data) != 4:
            return None
        flags = struct.unpack('<I', a.data)[0]
        b = o.chunk.find(0x7231)
        if b is not None and b.data is not None and len(b.data) == 4 and struct.unpack('<I', b.data)[0] != flags:
            raise NotImplementedError('Position/Rotation/Scale (object %d): chunks 0x7230 and 0x7231 are different' % idx)
        # chunk 0x7232/0x03e8: a Matrix3 (the "inheritOffsetTM" that the CAT source names). Identity in each scene.
        x = o.chunk.find(0x7232)
        if x is not None and x.children is not None:
            mc = x.find(0x03e8)
            if mc is not None and mc.data is not None and len(mc.data) == 48:
                m = np.array(struct.unpack('<12f', mc.data), float).reshape(4, 3)
                if np.abs(m - mm.ident()).max() > 1e-6:
                    raise NotImplementedError('Position/Rotation/Scale (object %d): inheritance offset matrix (chunk 0x7232) is not identity' % idx)
        return flags

    def _filter_parent(self, idx, tm, flags):
        """Remove from the parent matrix the parts that the node does not inherit."""
        if flags & ~INHERIT_SCL:
            raise NotImplementedError('Position/Rotation/Scale (object %d): inheritance flags 0x%x' % (idx, flags))
        if flags & INHERIT_SCL != INHERIT_SCL:
            raise NotImplementedError('Position/Rotation/Scale (object %d): inheritance flags 0x%x' % (idx, flags))
        a = tm[:3]
        if np.abs(a @ a.T - np.eye(3)).max() < 1e-5:
            return tm                                   # the parent has no scale
        self._flag('prs:scale not inherited and the parent has a scale', idx)
        parts = decomp_affine(tm)
        parts['S'] = np.eye(3)
        return comp_affine(parts)

    def apply(self, idx, t, tm):
        """Control::GetValue(t, &tm, valid, CTRL_RELATIVE): returns the new (4,3) matrix.
        For a Matrix3 controller (PRS, Link Constraint) tm is the parent matrix."""
        k = self.kind(idx)
        if k is None:
            raise NotImplementedError('%s (object %d): class not covered by maxstd' % (self.class_name(idx), idx))
        tm = np.array(tm, float)
        if k == 'prs':
            flags = self.inherit_flags(idx)
            if flags:
                tm = self._filter_parent(idx, tm, flags)
            for i in range(3):                          # position, rotation, scale
                r = self._ref(idx, i)
                if r >= 0:
                    tm = self.apply(r, t, tm)
            return tm
        if k == 'link':
            sub = self._ref(idx, 0)
            if sub < 0:
                raise NotImplementedError('Link Constraint (object %d): no transform controller' % idx)
            return self.apply(sub, t, self.link_parent_tm(idx, t))
        if k in ('posxyz', 'bezpos', 'noisepos'):
            return mm.pre_translate(tm, self.point3_value(idx, t))
        if k == 'poscon':
            tm[3] = self._poscon(idx, t)                # Matrix3::SetTrans
            return tm
        if k == 'poslist':
            conts, weights, avg = self._list(idx, t)
            for c, w in zip(conts, weights):
                if c >= 0 and w != 0.0:
                    prev = tm[3].copy()
                    tm = self.apply(c, t, tm)
                    tm[3] = prev + (tm[3] - prev) * self._avg(w, avg)
            return tm
        if k == 'euler':
            return mm.mul(mm.no_trans(self._euler(idx, t)), tm)
        if k == 'noiserot':
            return mm.mul(self._noise_rot(idx, t), tm)
        if k == 'linrot':
            return mm.pre_rotate(tm, self.quat_value(idx, t))
        if k == 'rotlist':
            return self._rotlist(idx, t, tm)
        if k == 'oricon':
            q, ct, local = self._oricon(idx, t)
            if local or ct < 1:
                return mm.pre_rotate(tm, q)
            parts = decomp_affine(tm)                   # keep all parts, replace the rotation
            parts['R'] = mm.quat_to_mat(q)[:3]
            return comp_affine(parts)
        if k in ('bezscale', 'scalexyz', 'noisescale'):
            s, q = self.scale_value(idx, t, True)
            return apply_scaling(tm, s, q)
        if k == 'scalelist':
            conts, weights, avg = self._list(idx, t)
            for c, w in zip(conts, weights):
                if w != 0.0 and c >= 0:
                    if self._avg(w, avg) != 1.0:
                        raise NotImplementedError('Scale List (object %d): a weight that is not 1' % idx)
                    self._flag('scale list:entries', idx)
                    tm = self.apply(c, t, tm)
            return tm
        raise TypeError('%s (object %d): apply needs a transform, position, rotation or scale controller' % (self.class_name(idx), idx))

    # ------------------------------------------------------------------ noise

    def _noise_params(self, idx):
        d = self._cache.get(('noise', idx))
        if d is None:
            o = self._obj(idx)

            def get(cid, fmt, default):
                c = o.chunk.find(cid)
                if c is None or c.data is None or len(c.data) != struct.calcsize(fmt):
                    return default
                v = struct.unpack(fmt, c.data)
                return v[0] if len(v) == 1 else v
            refs = o.refs or []
            if any(r >= 0 for r in refs[:2]):
                raise NotImplementedError('%s (object %d): ease or multiplier curves are not covered' % (o.cname, idx))
            # the base class Control writes chunk 0x8499: 0x3000 and 0x3001 out-of-range types,
            # 0x3003 flags (bit 0: the time range is off, A_ORT_DISABLED)
            flags = 0
            cb = o.chunk.find(0x8499)
            if cb is not None and cb.children is not None:
                fc = cb.find(0x3003)
                if fc is not None and fc.data is not None and len(fc.data) == 4:
                    flags = struct.unpack('<I', fc.data)[0]
            d = {'lim': [bool(get(0x0110 + i, '<i', 0)) for i in range(3)],
                 'frequency': get(0x0103, '<f', 0.5), 'roughness': get(0x0104, '<f', 0.0),
                 'seed': get(0x0105, '<i', 0), 'fractal': bool(get(0x0106, '<i', 1)),
                 'range': get(0x0107, '<ii', None), 'rampin': get(0x0108, '<i', 0), 'rampout': get(0x0109, '<i', 0),
                 'range_off': bool(flags & 1), 'ort': self._ort(idx),
                 'strength': refs[2] if len(refs) > 2 else -1}
            if flags & ~1:
                raise NotImplementedError('%s (object %d): control flags 0x%x' % (o.cname, idx, flags))
            if d['range'] is None and not d['range_off']:
                raise NotImplementedError('%s (object %d): no time range chunk 0x0107' % (o.cname, idx))
            self._cache[('noise', idx)] = d
        return d

    def _noise_at_time(self, d, t, seed, index):
        """BaseNoiseControl::NoiseAtTime with the float (32 bit) arithmetic of the C code."""
        ramp = _F(1.0)
        if not d['range_off']:
            start, end = d['range']
            if t < start + d['rampin']:
                u = _F(t - start) / _F(d['rampin'])
                ramp = ramp * (u * u * (_F(3.0) - _F(2.0) * u))
            if t > end - d['rampout']:
                u = _F(end - t) / _F(d['rampout'])
                ramp = ramp * (u * u * (_F(3.0) - _F(2.0) * u))
        v = _F(t) * _F(self.noise_time_factor) * _F(d['frequency']) + _F(perlin_perm(seed))
        if d['fractal']:
            res = _F(perlin_fbm1(v, float(_F(1.0) - _F(d['roughness'])), 2.0, 6))
        else:
            res = perlin_noise1(v)
        if d['lim'][index]:
            res = res + _F(0.5)
        return res * ramp

    def _noise(self, idx, t):
        """Noise times strength for each element: array of 1 (float) or 3 values."""
        k = self.kind(idx)
        n = 1 if k == 'noisefloat' else 3
        if self.noise_off:
            return np.zeros(n)
        d = self._noise_params(idx)
        # no file has a value of a noise controller: the port follows the SDK source, with no check
        self._flag('noise:' + self.class_name(idx), idx)
        if not d['range_off']:
            # StdControl::GetValue maps a time out of the range with the out-of-range type
            self._flag('noise:time range is on', idx)
            start, end = d['range']
            if t < start or t > end:
                ort = d['ort'][0] if t < start else d['ort'][1]
                if ort != ORT_CONSTANT:
                    raise NotImplementedError('%s (object %d): out-of-range type %s' % (self.class_name(idx), idx, ORT_NAMES.get(ort, ort)))
                t = start if t < start else end
        sc = d['strength']
        if sc < 0:
            raise NotImplementedError('%s (object %d): no strength controller' % (self.class_name(idx), idx))
        strength = [self.float_value(sc, t)] if n == 1 else list(self.point3_value(sc, t))
        with np.errstate(all='ignore'):
            return np.array([float(self._noise_at_time(d, t, d['seed'] + i, i) * _F(strength[i])) for i in range(n)])

    def _noise_rot(self, idx, t):
        """RotationNoiseControl::GetValueLocalTime: EulerToQuat of the three noise angles, as a matrix."""
        return mm.euler_to_mat(self._noise(idx, t), (0, 1, 2))

    # -------------------------------------------------------- link constraint

    def _link_params(self, idx):
        """(target node indices, start times). A target None is the world."""
        info = self._cache.get(('link', idx))
        if info is None:
            pb = self._ref(idx, 2)
            if pb < 0 or self.kind(pb) != 'pblock2':
                raise NotImplementedError('Link Constraint (object %d): no ParamBlock2 (file older than 3ds Max 4)' % idx)
            p = self.pblock2(pb)
            nodes = [n if (n is not None and n >= 0) else None for n in p.get(0, [])]
            times = list(p.get(2, []))
            info = (nodes, times)
            self._cache[('link', idx)] = info
        return info

    def _target_tm(self, node, t):
        return mm.ident() if node is None else np.asarray(self.node_tm(node, t), float)

    def _link_comp_tm(self, nodes, times, t, i):
        """LinkConstTransform::CompTM."""
        rtm = mm.ident()
        if i:
            pptm = self._link_comp_tm(nodes, times, times[i], i - 1)
            rtm = mm.mul(pptm, mm.inv(self._target_tm(nodes[i], times[i])))
        return mm.mul(rtm, self._target_tm(nodes[i], t))

    def link_parent_tm(self, idx, t):
        """LinkConstTransform::GetParentTM: the matrix that the Link Constraint gives to its PRS controller."""
        nodes, times = self._link_params(idx)
        ct = len(nodes)
        if ct != len(times) or ct < 1:
            return mm.ident()
        i = ct
        for j in range(ct):
            if times[j] > t:
                i = j - 1 if j else 0
                break
        if i > ct - 1:
            i = ct - 1
        return self._link_comp_tm(nodes, times, t, i)

    def link_target(self, idx, t):
        """Scene index of the node that the Link Constraint follows at t (None for the world)."""
        nodes, times = self._link_params(idx)
        if not nodes:
            return None
        i = len(nodes)
        for j in range(len(nodes)):
            if times[j] > t:
                i = j - 1 if j else 0
                break
        return nodes[min(i, len(nodes) - 1)]

    # --------------------------------------------------------------- key times

    def key_times(self, idx, _seen=None):
        """Sorted key times (ticks) of the controller and of its sub-controllers.
        A Link Constraint adds its link start times. The keys of target nodes are not included."""
        if _seen is None:
            _seen = set()
        if idx in _seen or idx < 0:
            return []
        _seen.add(idx)
        k = self.kind(idx)
        if k is None:
            raise NotImplementedError('%s (object %d): class not covered by maxstd' % (self.class_name(idx), idx))
        out = set()
        if k in ('bezfloat', 'bezpoint3', 'bezpos', 'bezscale', 'linrot'):
            out.update(self._keydata(idx)['times'])
        elif k in ('pblock2', 'pblock'):
            for par in self._pblock(idx):
                for kind, val, ref in par.entries:
                    if kind == 'ctl' and ref is not None and ref >= 0 and self._is_controller(ref) and self.handles(ref):
                        out.update(self.key_times(ref, _seen))
        else:
            if k == 'link':
                out.update(self._link_params(idx)[1])
            for r in self._refs(idx):
                if r >= 0 and (self.kind(r) in ('pblock2', 'pblock') or (self._is_controller(r) and self.handles(r))):
                    out.update(self.key_times(r, _seen))
        return sorted(out)

    def _is_controller(self, idx):
        cls = self.scene.objs[idx].cls
        return bool(cls) and (cls['super'] & 0xff00) == 0x9000

    # ------------------------------------------------------------- ParamBlock2

    def _pblock(self, idx):
        ps = self._pb.get(idx)
        if ps is None:
            k = self._need(idx, ('pblock2', 'pblock'), 'ParamBlock')
            ps = self._parse_pb2(idx) if k == 'pblock2' else self._parse_pb1(idx)
            self._pb[idx] = ps
        return ps

    def _pb2_value(self, d, pos, root, flags, idx, pid):
        fb = d[pos]
        pos += 1
        if root in PB2_REF_TYPES:
            if flags & (P_NO_REF | P_OWNERS_REF | P_SUBTEX):
                if fb & 0x40:
                    return 'idx', struct.unpack_from('<i', d, pos)[0], pos + 4
                return 'none', None, pos
            return 'ref', None, pos
        if root in (8, 9):
            if not fb & 0x40:
                return 'none', None, pos
            n = struct.unpack_from('<i', d, pos)[0]
            pos += 4
            if n > 0:
                return 'str', d[pos:pos + n].decode('utf-16-le').rstrip('\0'), pos + n
            return 'str', None, pos
        if root == 16:
            return 'bitmap', bool(d[pos]) if fb & 0x40 else None, pos + (1 if fb & 0x40 else 0)
        if not fb & 0x40:
            # an animatable parameter with no constant: its value is in a reference slot
            return ('ctl' if fb & 0x80 else 'none'), None, pos
        if root in PB2_FLOAT_TYPES:
            return 'val', struct.unpack_from('<f', d, pos)[0], pos + 4
        if root in PB2_INT_TYPES:
            return 'val', struct.unpack_from('<i', d, pos)[0], pos + 4
        if root in PB2_POINT3_TYPES:
            return 'val', np.array(struct.unpack_from('<3f', d, pos), float), pos + 12
        if root in (22, 23):
            return 'val', np.array(struct.unpack_from('<4f', d, pos), float), pos + 16
        if root == 30:
            return 'val', np.array(struct.unpack_from('<2f', d, pos), float), pos + 8
        if root == 20:      # Matrix3: 12 floats and the identity flags
            return 'val', np.array(struct.unpack_from('<12f', d, pos), float).reshape(4, 3), pos + 52
        raise NotImplementedError('ParamBlock2 (object %d): parameter %d has type %d, which has no decoder' % (idx, pid, root))

    def _parse_pb2(self, idx):
        o = self._obj(idx)
        refs = o.refs or []
        slot = 0
        out = []
        for c in o.chunk.children or []:
            if c.id == 0x000e:
                raise NotImplementedError('ParamBlock2 (object %d): parameter chunk 0x000e has no decoder; the scenes have none' % idx)
            if c.id != 0x100e:
                continue
            d = c.data
            if d is None:
                raise NotImplementedError('ParamBlock2 (object %d): parameter chunk 0x100e is a container' % idx)
            pid, typ, flags = struct.unpack_from('<hiq', d, 0)
            root = typ & 0x7ff
            pos = 14
            is_tab = bool(typ & TYPE_TAB)
            if is_tab:
                n = struct.unpack_from('<i', d, pos + 1)[0]
                pos += 5
            else:
                n = 1
            entries = []
            for _ in range(n):
                kind, val, pos = self._pb2_value(d, pos, root, flags, idx, pid)
                ref = None
                if kind in ('ref', 'ctl'):
                    # each owned reference and each controller has the next reference slot of the block
                    ref = refs[slot] if slot < len(refs) else -1
                    slot += 1
                entries.append((kind, val, ref))
            if pos != len(d):
                raise NotImplementedError('ParamBlock2 (object %d): parameter %d (type 0x%x) has %d bytes, decoder used %d'
                                          % (idx, pid, typ, len(d), pos))
            out.append(PBParam(pid, typ, flags, is_tab, entries))
        if slot != len(refs) and refs:
            raise NotImplementedError('ParamBlock2 (object %d): %d reference slots, the object has %d references' % (idx, slot, len(refs)))
        return out

    def _parse_pb1(self, idx):
        """ParamBlock of 3ds Max 1 to 3: chunk 0x0002 for each parameter (0x0003 index,
        0x0100 float, 0x0101 int, 0x0102 Point3 or colour, 0x0104 bool, 0x0200 controller)."""
        o = self._obj(idx)
        refs = o.refs or []
        out = []
        slot = 0
        for c in o.chunk.children or []:
            if c.id != 0x0002 or c.children is None:
                continue
            ic = c.find(0x0003)
            pid = struct.unpack('<i', ic.data)[0]
            entry = None
            typ = 0
            for cc in c.children:
                if cc.id == 0x0100:
                    entry, typ = ('val', struct.unpack('<f', cc.data)[0], None), 0
                elif cc.id in (0x0101, 0x0104):
                    entry, typ = ('val', struct.unpack('<i', cc.data)[0], None), (1 if cc.id == 0x0101 else 4)
                elif cc.id == 0x0102:
                    entry, typ = ('val', np.array(struct.unpack('<3f', cc.data), float), None), 3
                elif cc.id == 0x0200:
                    entry = ('ctl', None, refs[slot] if slot < len(refs) else -1)
                    slot += 1
                elif cc.id not in (0x0003, 0x0004):
                    raise NotImplementedError('ParamBlock (object %d): parameter %d has chunk 0x%04x, which has no decoder' % (idx, pid, cc.id))
            if entry is None:
                raise NotImplementedError('ParamBlock (object %d): parameter %d has no value chunk' % (idx, pid))
            out.append(PBParam(pid, typ, P_ANIMATABLE if c.find(0x0004) is not None else 0, False, [entry]))
        if slot != len(refs):
            raise NotImplementedError('ParamBlock (object %d): %d controllers, the object has %d references' % (idx, slot, len(refs)))
        return out

    @staticmethod
    def _pb_entry_value(entry):
        kind, val, ref = entry
        if kind == 'ref':
            return ref if ref is not None else -1
        return val          # 'ctl': None, the value comes from pblock2_controller

    def pblock2(self, idx):
        """Decoded ParamBlock2 (or old ParamBlock): param id -> value. A Tab parameter is a list.
        A node or reference value is a scene index (-1 for none). A parameter that a controller
        animates has the value None: use pblock2_controller and float_value, or pblock2_float."""
        out = {}
        for p in self._pblock(idx):
            vals = [self._pb_entry_value(e) for e in p.entries]
            out[p.id] = vals if p.is_tab else vals[0]
        return out

    def pblock2_params(self, idx):
        """The list of PBParam objects (id, type, flags, entries) of a ParamBlock2."""
        return self._pblock(idx)

    def pblock2_owner(self, idx):
        """(owner scene index, class name or Class_ID text, block id) from the header of a ParamBlock2."""
        o = self._obj(idx)
        h = o.chunk.find(0x0009)
        if h is not None and h.data is not None and len(h.data) == 16:
            cls, bid, _, _, _, owner = struct.unpack('<iHHHHi', h.data)
            name = self.scene.classes[cls]['name'] if 0 <= cls < len(self.scene.classes) else '?%d' % cls
            return owner, name, bid
        h = o.chunk.find(0x000b)
        if h is not None and h.data is not None and len(h.data) == 24:
            a, b, sup, bid, _, _, _, owner = struct.unpack('<IIIHHHHi', h.data)
            return owner, '%08x:%08x' % (a, b), bid
        return None, None, None

    def _pb_param(self, idx, pid):
        for p in self._pblock(idx):
            if p.id == pid:
                return p
        return None

    def pblock2_count(self, idx, pid):
        """Number of entries of a Tab parameter (0 when the block has no such parameter)."""
        p = self._pb_param(idx, pid)
        return len(p.entries) if p is not None else 0

    def pblock2_controller(self, idx, pid, tab_index=0):
        """Scene index of the controller that animates the parameter, or None.
        The CAT classes also keep other objects in the reference slot of an animatable parameter:
        the function returns the scene index of that object."""
        p = self._pb_param(idx, pid)
        if p is None or tab_index >= len(p.entries):
            return None
        kind, val, ref = p.entries[tab_index]
        if kind == 'ctl' and ref is not None and ref >= 0:
            return ref
        return None

    def pblock2_float(self, idx, pid, t, tab_index=0):
        """Value of a float or integer parameter at time t (evaluates its controller)."""
        p = self._pb_param(idx, pid)
        if p is None or tab_index >= len(p.entries):
            raise KeyError('ParamBlock2 (object %d): no parameter %d[%d]' % (idx, pid, tab_index))
        kind, val, ref = p.entries[tab_index]
        if kind == 'val':
            return float(val)
        if kind == 'ctl' and ref is not None and ref >= 0:
            return self.float_value(ref, t)
        raise NotImplementedError('ParamBlock2 (object %d): parameter %d[%d] has no value in the file' % (idx, pid, tab_index))


def comp_affine_rot(tm):
    """The rotation matrix of decomp_affine(tm).q as a (4,3) matrix."""
    r = mm.ident()
    r[:3] = decomp_affine(tm)['R']
    return r
