"""Evaluator for the CAT rig classes (cat.dlc) of a 3ds Max scene.

The rules are from the CAT source of the public 3ds Max SDK samples
(maxsdk/samples/CAT/src/CATControls). Each method has the name of the C++
function that it follows. Only the code paths that the MP7 scenes use are
here; another path raises NotImplementedError.
"""
import math
import struct
import numpy as np
import maxmath as mm

X, Y, Z = 0, 1, 2
SETUPMODE, NORMAL = 0, 1

# CATControl flags
CCFLAG_ANIM_STRETCHY = 1 << 12
CNCFLAG_IMPOSE_POS_LIMITS = 1 << 21
CNCFLAG_IMPOSE_ROT_LIMITS = 1 << 22
CNCFLAG_IMPOSE_SCL_LIMITS = 1 << 23
CNCFLAG_INHERIT_ANIM_POS = 1 << 25
CNCFLAG_INHERIT_ANIM_ROT = 1 << 26
CNCFLAG_INHERIT_ANIM_SCL = 1 << 27
CNCFLAG_INHERIT_ANIM_ALL = CNCFLAG_INHERIT_ANIM_POS | CNCFLAG_INHERIT_ANIM_ROT | CNCFLAG_INHERIT_ANIM_SCL

# CATClipValue flags
CLIP_FLAG_HAS_TRANSFORM = 1 << 9
CLIP_FLAG_INHERIT_POS = 1 << 20
CLIP_FLAG_INHERIT_ROT = 1 << 21
CLIP_FLAG_INHERIT_SCL = 1 << 22
CLIP_FLAG_RELATIVE_TO_SETUPPOSE = 1 << 27
CLIP_FLAG_SETUPPOSE_CONTROLLER = 1 << 28
CLIP_FLAG_KEYFREEFORM = 1 << 30
CLIP_FLAG_DISABLE_LAYERS = 1 << 31

LAYER_RELATIVE, LAYER_ABSOLUTE, LAYER_IGNORE, LAYER_RELATIVE_WORLD, LAYER_CATMOTION = range(5)
LAYER_DISABLE = 1 << 4

# LimbData2 flags
LIMBFLAG_ISARM = 1 << 1
LIMBFLAG_ISLEG = 1 << 2
LIMBFLAG_LEFT = 1 << 3
LIMBFLAG_RIGHT = 1 << 5
LIMBFLAG_LOCKED_FK = 1 << 8
LIMBFLAG_LOCKED_IK = 1 << 9
LIMBFLAG_FFB_WORLDZ = 1 << 12

SPINEFLAG_FKSPINE = 1 << 3

ONES = np.ones(3)


def u32(b, o=0):
    return struct.unpack_from('<I', b, o)[0]


def i32(b, o=0):
    return struct.unpack_from('<i', b, o)[0]


def f32s(b, n, o=0):
    return np.array(struct.unpack_from('<%df' % n, b, o), float)


def cos_limit(v):
    return max(min(v, 1.0), -1.0)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v * 0.0


def angaxis_mat(axis, angle):
    """Matrix of an SDK AngAxis (left-hand rule)."""
    h = angle * 0.5
    s = math.sin(h)
    a = np.asarray(axis, float)
    return mm.quat_to_mat((a[0] * s, a[1] * s, a[2] * s, math.cos(h)))


def rotate_matrix(m, axis, angle):
    """RotateMatrix(Matrix3&, AngAxis): mat = mat * rotation. The translation turns too."""
    if not np.all(np.isfinite(axis)) or np.linalg.norm(axis) == 0:
        return m.copy()
    return mm.mul(m, angaxis_mat(axis, angle))


def rot_mat(m, axis, angle):
    """RotMat: rotate and keep the position."""
    r = rotate_matrix(m, axis, angle)
    r[3] = m[3]
    return r


def pre_rot(m, f, a):
    r = m.copy()
    r[:3] = f(a)[:3] @ m[:3]
    return r


def rotate_matrix_to_align_with_vector(tm, vector, axis):
    pos = tm[3].copy()
    mv = tm[axis]
    ang = math.acos(cos_limit(float(np.dot(vector, mv))))
    if ang > 0.0:
        ax = unit(np.cross(vector, mv))
        tm = rotate_matrix(tm, ax, ang)
    tm = tm.copy()
    tm[3] = pos
    return tm


def decomp(m):
    """decomp_affine for a matrix with no shear: translation, rotation quaternion, scale."""
    k = np.array([np.linalg.norm(m[0]), np.linalg.norm(m[1]), np.linalg.norm(m[2])])
    return m[3].copy(), mm.mat_to_quat(m), k


def blend_mat(tm1, tm2, ratio):
    if ratio == 1.0:
        return tm2.copy()
    if ratio == 0.0:
        return tm1.copy()
    t1, q1, k1 = decomp(tm1)
    t2, q2, k2 = decomp(tm2)
    t = t1 + (t2 - t1) * ratio
    q = mm.quat_slerp(q1, q2, ratio)
    k = k1 + (k2 - k1) * ratio
    r = mm.quat_to_mat(q)
    r = mm.pre_scale(r, k)
    r[3] = t
    return r


def blend_rot(tm1, tm2, ratio):
    if ratio == 1.0:
        r = tm1.copy()
        r[:3] = tm2[:3]
        return r
    if ratio == 0.0:
        return tm1.copy()
    q = mm.quat_slerp(mm.mat_to_quat(tm1), mm.mat_to_quat(tm2), ratio)
    r = mm.quat_to_mat(q)
    r[3] = tm1[3]
    return r


def blend_float(a, b, r):
    return a + (b - a) * r


class CatError(Exception):
    pass


class Rig:
    """All CAT objects of one scene, and the evaluation of node matrices."""

    def __init__(self, scene, std):
        self.scene = scene
        self.std = std
        self.objs = {}
        self.t = None
        self.node_cache = {}
        self._stack = []
        self.node_of_ctrl = {}
        for o in scene.nodes():
            if o.refs and o.refs[0] >= 0:
                self.node_of_ctrl.setdefault(o.refs[0], o.index)
        classes = {
            'HubTrans': Hub, 'CATSpineTrans2': SpineTrans2, 'CATCollarBone': CollarBoneTrans,
            'CATBoneData': BoneData, 'CATBoneSegTrans': BoneSegTrans, 'PalmTrans': PalmTrans2,
            'CATDigitSegTrans': DigitSegTrans, 'ArbBone': ArbBoneTrans, 'IKTarget Trans': IKTargTrans,
            'CATFootTrans2': FootTrans2, 'CATLimbData2': LimbData2, 'CATSpineData2': SpineData2,
            'DigitData': DigitData, 'CATParentTrans': CATParentTrans, 'LayerMatrix3': ClipMatrix3,
            'LayerFloat': ClipFloat, 'LayerWeights': ClipWeights, 'LayerRoot': ClipRoot,
            'LayerInfo': NLAInfo, 'CATGizmoTransform': GizmoTransform, 'CATWeight': CATWeight,
        }
        for o in scene.objs:
            c = classes.get(o.cname)
            if c is not None and o.cls['dll'] >= 0 and scene.dlls[o.cls['dll']]['file'].lower() == 'cat.dlc':
                self.objs[o.index] = c(self, o)
        for c in self.objs.values():
            c.link()

    def get(self, idx):
        if idx is None or idx < 0:
            return None
        return self.objs.get(idx)

    def set_time(self, t):
        self.t = t

    # --- nodes -------------------------------------------------------------
    def node_ctrl(self, node_idx):
        o = self.scene.objs[node_idx]
        return o.refs[0] if o.refs else -1

    def node_parent(self, node_idx):
        p = self.scene.objs[node_idx].parent
        if p is None or p < 0:
            return None
        po = self.scene.objs[p]
        if po.cname != 'Node':
            return None
        return p

    def node_tm(self, node_idx, t):
        """World matrix of a node. A controller can ask for another time (a link constraint
        does), so each cache has the time in its key."""
        key = (node_idx, t)
        r = self.node_cache.get(key)
        if r is not None:
            return r
        if key in self._stack:
            raise CatError('cycle at node %s' % self.scene.objs[node_idx].name)
        prev = self.t
        self.t = t
        self._stack.append(key)
        try:
            p = self.node_parent(node_idx)
            ptm = self.node_tm(p, t) if p is not None else mm.ident()
            ctl = self.node_ctrl(node_idx)
            tm = self.ctrl_value(ctl, t, ptm)
        finally:
            self._stack.pop()
            self.t = prev
        self.node_cache[key] = tm
        return tm

    def ctrl_value(self, ctl, t, ptm):
        """Control::GetValue(t, &tm, valid, CTRL_RELATIVE) of a Matrix3 controller."""
        c = self.objs.get(ctl)
        if c is not None:
            return c.get_value(t, ptm.copy())
        return self.std.apply(ctl, t, ptm.copy())

    def obj_tm(self, node_idx, t):
        """INode::GetObjTMAfterWSM: object offset * node matrix."""
        tm = self.node_tm(node_idx, t)
        o = self.scene.objs[node_idx]
        pos = o.chunk.find(0x096A)
        rot = o.chunk.find(0x096B)
        scl = o.chunk.find(0x096C)
        off = mm.ident()
        if scl is not None:
            s = f32s(scl.data, 3)
            if not np.allclose(s, 1.0, atol=1e-6):
                off = mm.mul(off, mm.scale_mat(s))
        if rot is not None:
            q = f32s(rot.data, 4)
            if not np.allclose(np.abs(q), (0, 0, 0, 1), atol=1e-6):
                off = mm.mul(off, mm.quat_to_mat(q))
        if pos is not None:
            p = f32s(pos.data, 3)
            off = mm.translate(off, p)
        return mm.mul(off, tm)


class Base:
    def __init__(self, rig, o):
        self.rig = rig
        self.o = o
        self.idx = o.index
        self.refs = o.refs or []
        self._c = {}

    @property
    def c(self):
        d = self._c.get(self.rig.t)
        if d is None:
            d = self._c[self.rig.t] = {}
        return d

    def link(self):
        pass

    def ref(self, i):
        return self.refs[i] if i < len(self.refs) else -1

    def pb(self, ref_index=0):
        if not hasattr(self, '_pb'):
            self._pb = self.rig.std.pblock2(self.ref(ref_index))
        return self._pb

    def pbref(self, pid, ref_index=0):
        """A reference value of the parameter block: a scene index or None."""
        v = self.pb(ref_index).get(pid)
        if isinstance(v, (list, tuple)):
            raise TypeError('parameter %d is a table' % pid)
        if v is None or (isinstance(v, int) and v < 0):
            v = self.rig.std.pblock2_controller(self.ref(ref_index), pid)
        return v if v is not None and v >= 0 else None

    def pbtab(self, pid, ref_index=0):
        """A table of references: a list of scene indices."""
        v = self.pb(ref_index).get(pid)
        if v is None:
            return []
        out = []
        for i, x in enumerate(v):
            if x is None or (isinstance(x, int) and x < 0):
                x = self.rig.std.pblock2_controller(self.ref(ref_index), pid, i)
            out.append(x if x is not None and x >= 0 else None)
        return out

    @property
    def cat_parent(self):
        return self.rig.catparent


class CATParentTrans(Base):
    def __init__(self, rig, o):
        super().__init__(rig, o)
        ch = o.chunk
        self.catunits = f32s(ch.find(3).data, 1)[0]
        self.catmode = i32(ch.find(4).data)
        self.lengthaxis = i32(ch.find(9).data)
        self.node = u32(ch.find(6).data)
        self.prs = self.ref(1)
        self.layerroot = self.ref(2)
        rig.catparent = self

    def get_value(self, t, val):
        return self.rig.std.apply(self.prs, t, val)

    def node_tm_noscale(self, t):
        return mm.no_scale(self.rig.node_tm(self.node, t))


class ClipRoot(Base):
    def __init__(self, rig, o):
        super().__init__(rig, o)
        self.nlayers = i32(o.chunk.find(1).data)
        self.solo = i32(o.chunk.find(0x0e).data) if o.chunk.find(0x0e) is not None else -1
        if self.nlayers != 1:
            raise NotImplementedError('CAT rig with %d layers' % self.nlayers)
        if self.solo not in (-1,):
            raise NotImplementedError('solo layer')
        rig.cliproot = self

    def layer(self, i):
        return self.rig.get(self.refs[i])


class NLAInfo(Base):
    def __init__(self, rig, o):
        super().__init__(rig, o)
        self.flags = u32(o.chunk.find(2).data)
        self.method = self.flags & 7
        self.weight = self.ref(0)
        self.timewarp = self.ref(1)
        self.transform = self.ref(2)
        if self.timewarp >= 0:
            raise NotImplementedError('layer time warp')
        if self.method != LAYER_ABSOLUTE or self.flags & LAYER_DISABLE:
            raise NotImplementedError('layer method %d flags %x' % (self.method, self.flags))

    def get_transform(self, t):
        if 'tm' in self.c:
            return self.c['tm']
        tm = mm.ident()
        if self.transform >= 0:
            std = self.rig.std
            tr = self.rig.scene.objs[self.transform]
            if tr.cname == 'Position/Rotation/Scale':
                tm = std.apply(tr.refs[0], t, tm)
                tm = std.apply(tr.refs[1], t, tm)
            else:
                tm = mm.no_scale(std.apply(self.transform, t, tm))
        self.c['tm'] = tm
        return tm

    def get_scale(self, t):
        if self.transform < 0:
            return ONES.copy()
        tr = self.rig.scene.objs[self.transform]
        if tr.cname == 'Position/Rotation/Scale' and len(tr.refs) > 2 and tr.refs[2] >= 0:
            return scale3(self.rig.std.scale_value(tr.refs[2], t))
        return ONES.copy()


def scale3(v):
    if isinstance(v, tuple):
        v = v[0]
    return np.asarray(v, float)


class ClipValue(Base):
    def __init__(self, rig, o):
        super().__init__(rig, o)
        ch = o.chunk
        self.nlayers = i32(ch.find(1).data)
        self.flags = u32(ch.find(3).data)
        self.layers = self.refs[:self.nlayers]
        self.setup_ctrl = self.ref(self.nlayers)
        if self.flags & (CLIP_FLAG_RELATIVE_TO_SETUPPOSE | CLIP_FLAG_SETUPPOSE_CONTROLLER | CLIP_FLAG_KEYFREEFORM):
            raise NotImplementedError('clip value flags %x' % self.flags)

    def test(self, f):
        return (self.flags & f) == f


class ClipWeights(ClipValue):
    pass


class ClipFloat(ClipValue):
    def get_value(self, t):
        if self.cat_parent.catmode == SETUPMODE:
            raise NotImplementedError('setup mode')
        if self.flags & CLIP_FLAG_DISABLE_LAYERS:
            return 0.0
        # one absolute layer with the weight 1: val = 0 + (layer - 0) * 1
        return float(self.rig.std.float_value(self.layers[0], t))


class ClipMatrix3(ClipValue):
    def __init__(self, rig, o):
        super().__init__(rig, o)
        sv = o.chunk.find(8)
        self.setup = f32s(sv.data, 12).reshape(4, 3) if sv is not None and len(sv.data) >= 48 else mm.ident()

    def layer_ctrl(self):
        return self.layers[0]

    def inherit_flags(self):
        sc = self.rig.scene
        ctl = self.layer_ctrl()
        o = sc.objs[ctl]
        if o.cname == 'Link Constraint':
            ctl = o.refs[0]
        f = self.rig.std.inherit_flags(ctl)
        return int(f or 0)

    def scale_controller(self):
        sc = self.rig.scene
        o = sc.objs[self.layer_ctrl()]
        if o.cname == 'Link Constraint':
            o = sc.objs[o.refs[0]]
        if o.cname == 'Position/Rotation/Scale':
            return o.refs[2] if len(o.refs) > 2 else -1
        raise NotImplementedError('layer controller %s' % o.cname)

    def get_transformation(self, t, tm_orig_parent, tm_value, p3_parent_scale, p3_local_scale):
        """CATClipMatrix3::GetTransformation for one absolute layer with the weight 1."""
        if self.flags & CLIP_FLAG_DISABLE_LAYERS:
            return tm_value, p3_local_scale
        rig = self.rig
        std = rig.std
        info = rig.cliproot.layer(0)
        ctl = self.layer_ctrl()
        inh_all = self.test(CLIP_FLAG_INHERIT_POS | CLIP_FLAG_INHERIT_ROT | CLIP_FLAG_INHERIT_SCL)
        # CalculateAbsLayerParent
        if self.test(CLIP_FLAG_HAS_TRANSFORM):
            tm_layer = mm.mul(tm_value, info.get_transform(t))
        elif not inh_all:
            raise NotImplementedError('partial inheritance of the layer transform')
        else:
            tm_layer = tm_value.copy()
        p3_layer_scale = np.array(p3_parent_scale, float)
        prelayer = tm_layer.copy()
        tm_layer = std.apply(ctl, t, tm_layer)
        if not inh_all:
            raise NotImplementedError('partial inheritance')
        iflags = self.inherit_flags()
        if iflags & 0x38:
            # A layer controller that does not inherit a rotation: the layer transform is added after it.
            pos = tm_layer[3].copy()
            tm_layer = mm.mul(tm_layer, info.get_transform(t))
            tm_layer[3] = pos
        if not np.allclose(p3_layer_scale, 1.0, atol=1e-7):
            localpos = mm.mul(tm_layer, mm.inv(prelayer))[3]
            localpos = mm.ptrans(localpos, mm.mul(mm.scale_mat(p3_parent_scale), prelayer))
            tm_layer[3] = localpos
        # ApplyInheritance(t, p3LayerScale, i): a set bit means that the axis does not inherit the scale.
        for i in range(3):
            if iflags & (0x40 << i):
                p3_layer_scale[i] = 1.0
        sc = self.scale_controller()
        if sc >= 0:
            s = scale3(std.scale_value(sc, t))
            if not np.array_equal(s, ONES):
                tm_layer = mm.mul(mm.inv(mm.scale_mat(s)), tm_layer)
                p3_layer_scale = s * p3_layer_scale
        else:
            tr, q, k = decomp(tm_layer)
            p3_layer_scale = k
            tm_layer = mm.quat_to_mat(q)
            tm_layer[3] = tr
        if self.test(CLIP_FLAG_HAS_TRANSFORM):
            ls = info.get_scale(t)
            if not np.allclose(ls, 1.0, atol=1e-7):
                old = info.get_transform(t)[3]
                tm_layer[3] = old + (tm_layer[3] - old) * ls
        return tm_layer, p3_layer_scale

    def get_scale(self, t):
        tm, s = self.get_transformation(t, mm.ident(), mm.ident(), ONES, ONES.copy())
        return s


class CATWeight(Base):
    """A weight graph. The value at the ratio r (0 to 1)."""

    def value(self, ratio):
        pb = self.pb()
        k1 = float(pb.get(0, 0.0))
        k2 = float(pb.get(2, 1.0))
        # The graph is a Bezier curve between the two keys. A straight line is used here.
        return k1 + (k2 - k1) * ratio


class CATControl(Base):
    CTRL_CHUNK = None   # the chunk that has the CATNodeControl or CATControl data

    def __init__(self, rig, o):
        super().__init__(rig, o)
        cc = self.find_catcontrol(o.chunk)
        self.ccflags = u32(cc.find(2).data)
        b = cc.find(6)
        self.bone_id = i32(b.data) if b is not None else -1
        nm = cc.find(3)
        self.cname_text = nm.data.decode('latin-1') if nm is not None else ''

    def find_catcontrol(self, ch):
        # A CATControl chunk set has the chunks 5, 1, 2, 3, 4, 6.
        for c in ch.children or []:
            if c.children is not None:
                ids = [x.id for x in c.children]
                if 2 in ids and 4 in ids and 5 in ids and all(x.children is None for x in c.children):
                    self._nodectrl_chunk = ch
                    return c
                r = self.find_catcontrol(c)
                if r is not None:
                    return r
        return None

    def test_cc(self, f):
        return (self.ccflags & f) == f

    @property
    def catmode(self):
        return self.cat_parent.catmode

    @property
    def catunits(self):
        return self.cat_parent.catunits

    @property
    def length_axis(self):
        return self.cat_parent.lengthaxis


class CATNodeControl(CATControl):
    LAYERTRANS_REF = None

    def __init__(self, rig, o):
        super().__init__(rig, o)
        nc = self._nodectrl_chunk
        d = nc.find(0x0a)
        self.obj_dim = f32s(d.data, 3) if d is not None else np.zeros(3)
        self.node = rig.node_of_ctrl.get(self.idx)
        self.layer_trans = None
        if self.ccflags & (CNCFLAG_IMPOSE_POS_LIMITS | CNCFLAG_IMPOSE_ROT_LIMITS | CNCFLAG_IMPOSE_SCL_LIMITS):
            raise NotImplementedError('bone limits on %s' % o.cname)
        if self.ccflags & CCFLAG_ANIM_STRETCHY:
            raise NotImplementedError('stretchy bone')

    def link(self):
        if self.LAYERTRANS_REF is not None:
            self.layer_trans = self.rig.get(self.ref(self.LAYERTRANS_REF))

    # --- helpers -----------------------------------------------------------
    def get_node_tm(self, t):
        if self.node is None:
            return self.c['world']
        return self.rig.node_tm(self.node, t)

    def get_parent_node(self):
        return self.rig.node_parent(self.node) if self.node is not None else None

    def find_parent_cat_node_control(self):
        return None

    def get_parent_cat_node_control(self, cat_parent=False):
        if cat_parent:
            return self.find_parent_cat_node_control()
        pn = self.get_parent_node()
        if pn is not None:
            c = self.rig.get(self.rig.node_ctrl(pn))
            return c if isinstance(c, CATNodeControl) else None
        if self.node is not None:
            return None     # the parent is the scene root
        return self.find_parent_cat_node_control()

    def get_parent_tm(self, t):
        p = self.get_parent_cat_node_control()
        if p is not None:
            return p.get_node_tm(t)
        pn = self.get_parent_node()
        if pn is not None:
            return self.rig.node_tm(pn, t)
        return mm.ident()

    def get_bone_length(self):
        return float(self.obj_dim[self.length_axis])

    def get_local_scale(self, t):
        if self.catmode != SETUPMODE and self.layer_trans is not None:
            return self.layer_trans.get_scale(t)
        return ONES.copy()

    def get_bone_dimensions(self, t):
        return self.obj_dim * self.get_local_scale(t)

    def tm_bone_world(self, t):
        return self.c['world']

    def world_scale(self, t):
        return self.c.get('scale', ONES)

    # --- evaluation --------------------------------------------------------
    def base_apply_child_offset(self, t, tm):
        off = np.zeros(3)
        off[self.length_axis] = self.obj_dim[self.length_axis]
        return mm.pre_translate(tm, off)

    def apply_child_offset(self, t, tm):
        return self.base_apply_child_offset(t, tm)

    def apply_setup_offset(self, t, tm_orig_parent, tm_world, scale):
        lt = self.layer_trans
        if lt is not None and (self.catmode == SETUPMODE or lt.test(CLIP_FLAG_RELATIVE_TO_SETUPPOSE)):
            raise NotImplementedError('setup pose')
        return tm_world, scale

    def base_calc_parent_transform(self, t, tm):
        p = self.get_parent_cat_node_control()
        if p is not None:
            tm = p.apply_child_offset(t, tm)
        return self.calc_inheritance(t, tm)

    def calc_parent_transform(self, t, tm):
        return self.base_calc_parent_transform(t, tm)

    def base_calc_inheritance(self, t, tm):
        ps = np.array([np.linalg.norm(tm[0]), np.linalg.norm(tm[1]), np.linalg.norm(tm[2])])
        tm = tm.copy()
        for i in range(3):
            if ps[i] > 0:
                tm[i] /= ps[i]
        if np.allclose(ps, 1.0, atol=1e-7):
            ps = ONES.copy()
        if self.layer_trans is None:
            return tm, ps
        if self.catmode == SETUPMODE:
            raise NotImplementedError('setup mode')
        if not self.test_cc(CNCFLAG_INHERIT_ANIM_POS):
            tm = mm.no_trans(tm)
        if not self.test_cc(CNCFLAG_INHERIT_ANIM_ROT):
            tm = mm.no_rot(tm)
        if not self.test_cc(CNCFLAG_INHERIT_ANIM_SCL):
            ps = ONES.copy()
        return tm, ps

    def calc_inheritance(self, t, tm):
        return self.base_calc_inheritance(t, tm)

    def base_calc_world_transform(self, t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale):
        if self.catmode != SETUPMODE and self.layer_trans is not None:
            return self.layer_trans.get_transformation(t, tm_orig_parent, tm_world, p3_parent_scale, p3_local_scale)
        return tm_world, p3_local_scale

    def calc_world_transform(self, t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale):
        return self.base_calc_world_transform(t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale)

    def base_get_value(self, t, val):
        tm_orig_parent = val.copy()
        tm_world, p3_parent_scale = self.calc_parent_transform(t, val.copy())
        tm_bone_parent = tm_world.copy()
        p3_local_scale = ONES.copy()
        tm_world, p3_local_scale = self.apply_setup_offset(t, tm_orig_parent, tm_world, p3_local_scale)
        tm_world, p3_local_scale = self.calc_world_transform(t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale)
        self.c['local'] = mm.mul(tm_world, mm.inv(tm_bone_parent))
        self.c['world'] = tm_world.copy()
        self.c['scale'] = p3_local_scale * p3_parent_scale
        self.c['last_parent'] = tm_orig_parent
        s = p3_local_scale * p3_parent_scale
        if np.array_equal(s, ONES):
            return tm_world
        return mm.pre_scale(tm_world, s)

    def get_value(self, t, val):
        return self.base_get_value(t, val)


class Hub(CATNodeControl):
    LAYERTRANS_REF = 1
    PB_INSPINE, PB_SPINE_TAB, PB_LIMB_TAB = 8, 9, 11

    def link(self):
        super().link()
        self.dangle = self.rig.get(self.ref(2))

    def apply_child_offset(self, t, tm):
        return tm       # Hub.h: an empty override

    def limbs(self):
        return [self.rig.get(i) for i in self.pbtab(self.PB_LIMB_TAB)]

    def calc_parent_transform(self, t, tm):
        p = self.get_parent_cat_node_control()
        if isinstance(p, SpineTrans2):
            sp = p.spine()
            if sp is not None and not sp.fk:
                raise NotImplementedError('procedural spine')
        return self.base_calc_parent_transform(t, tm)

    def calc_world_transform(self, t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale):
        tm_world, p3_local_scale = self.base_calc_world_transform(t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale)
        p = self.get_parent_cat_node_control()
        if isinstance(p, SpineTrans2):
            sp = p.spine()
            if sp is not None and not sp.fk:
                raise NotImplementedError('procedural spine')
        else:
            tm_world = self.apply_reach(t, tm_world, p3_local_scale)
        return tm_world, p3_local_scale

    def apply_reach(self, t, tm, hub_scale):
        limbs = [l for l in self.limbs() if l is not None]
        if not limbs:
            return tm
        tot = 0.0
        for l in limbs:
            w = l.reach_weight(t)
            tot += w
        if tot <= 0.0:
            return tm
        raise NotImplementedError('limb force feedback on a hub (weight %g)' % tot)


class SpineData2(CATControl):
    PB_HUBBASE, PB_HUBTIP, PB_SPINETRANS_TAB = 1, 2, 10

    def __init__(self, rig, o):
        super().__init__(rig, o)
        f = o.chunk.find(1)
        self.flags = u32(f.data) if f is not None and f.children is None else 0
        self.fk = bool(self.flags & SPINEFLAG_FKSPINE)


class SpineTrans2(CATNodeControl):
    LAYERTRANS_REF = 1
    PB_SPINEDATA = 0

    def spine(self):
        return self.rig.get(self.pbref(self.PB_SPINEDATA))

    def calc_world_transform(self, t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale):
        sp = self.spine()
        if sp is None:
            raise CatError('spine link with no spine data')
        if not sp.fk:
            raise NotImplementedError('procedural spine')
        return self.base_calc_world_transform(t, tm_orig_parent, p3_parent_scale, tm_world, p3_local_scale)


class CollarBoneTrans(CATNodeControl):
    LAYERTRANS_REF = 1
    PB_LIMBDATA = 0

    def limb(self):
        return self.rig.get(self.pbref(self.PB_LIMBDATA))

    def apply_child_offset(self, t, tm):
        tm = self.base_apply_child_offset(t, tm)
        pos = tm[3].copy()
        r = self.get_parent_tm(t).copy()
        ls = self.get_local_scale(t)
        if not np.array_equal(ls, ONES):
            r = mm.pre_scale(r, ls)
        r[3] = pos
        return r

    def calc_parent_transform(self, t, tm):
        tm, ps = self.base_calc_parent_transform(t, tm)
        limb = self.limb()
        if limb is not None:
            tm = pre_rot(tm, mm.roty, math.pi / 2 * limb.lmr)
            ps = np.array([ps[2], ps[1], ps[0]])
        return tm, ps


class LimbData2(CATControl):
    PB_HUB, PB_LIMBFLAGS, PB_CTRLCOLLARBONE, PB_CTRLPALM, PB_NUMBONES, PB_BONEDATATAB = 1, 7, 8, 9, 11, 12

    @property
    def runtime_flags(self):
        return self.c.get('runtime_flags', 0)

    @runtime_flags.setter
    def runtime_flags(self, v):
        self.c['runtime_flags'] = v

    def link(self):
        pb = self.pb()
        self.flags = int(pb.get(self.PB_LIMBFLAGS, 0))
        self.bones = [self.rig.get(i) for i in self.pbtab(self.PB_BONEDATATAB)]
        self.palm = self.rig.get(self.pbref(self.PB_CTRLPALM))
        self.collar = self.rig.get(self.pbref(self.PB_CTRLCOLLARBONE))
        self.hub = self.rig.get(self.pbref(self.PB_HUB))
        self.layer_retargeting = self.rig.get(self.ref(2))
        self.layer_ikfk = self.rig.get(self.ref(3))
        self.layer_ikpos = self.rig.get(self.ref(4))
        self.iktarget = self.ref(5) if self.ref(5) >= 0 else None
        self.layer_iktarget_trans = self.rig.get(self.ref(6))
        self.upnode = self.ref(7) if self.ref(7) >= 0 else None
        self.lmr = -1 if self.flags & LIMBFLAG_LEFT else (1 if self.flags & LIMBFLAG_RIGHT else 0)
        self.is_leg = bool(self.flags & LIMBFLAG_ISLEG)
        self.is_arm = bool(self.flags & LIMBFLAG_ISARM)

    def test(self, f):
        return ((self.flags | self.runtime_flags) & f) == f

    @property
    def num_bones(self):
        return len(self.bones)

    def bone(self, i):
        return self.bones[i] if 0 <= i < len(self.bones) else None

    def get_ikfk_ratio(self, t, boneid=-1):
        if self.test(LIMBFLAG_LOCKED_FK):
            return 1.0
        if self.test(LIMBFLAG_LOCKED_IK):
            return 0.0
        if self.layer_iktarget_trans is None and self.iktarget is None:
            return 1.0
        r = 0.0
        if self.layer_ikfk is not None:
            r = self.layer_ikfk.get_value(t)
        r = min(1.0, max(r, 0.0))
        if boneid < 0:
            return r
        ikpos = self.get_limb_ik_pos(t)
        if (boneid + 1.0) <= ikpos:
            return r
        if boneid >= ikpos:
            return 1.0
        ikpos -= boneid
        return blend_float(r, 1.0, 1.0 - ikpos)

    def get_force_feedback(self, t):
        if self.test(LIMBFLAG_LOCKED_IK) or self.catmode == SETUPMODE:
            return 0.0
        v = self.layer_retargeting.get_value(t) if self.layer_retargeting is not None else 0.0
        return min(1.0, max(v, 0.0))

    def reach_weight(self, t):
        r = self.get_ikfk_ratio(t)
        w = self.get_force_feedback(t) * (1.0 - r)
        if r >= 1.0 or w <= 0.0:
            return 0.0
        return w

    def get_limb_ik_pos(self, t):
        v = self.layer_ikpos.get_value(t) if self.layer_ikpos is not None else 0.0
        v = max(v, 0.0)
        hi = self.num_bones + (1 if self.palm is not None else 0)
        return min(v, hi)

    def get_limb_ik_length(self, t, frombone, limb_ik_pos=-1):
        numbones = self.num_bones
        ikpos = limb_ik_pos if limb_ik_pos > 0 else self.get_limb_ik_pos(t)
        last = int(ikpos)
        length = 0.0
        if last < ikpos:
            last += 1
        if 'fkpos' not in self.c or last >= len(self.c['fkpos']):
            self.get_tm_fk_target(t, None)
        if ikpos > numbones:
            palm = self.palm
            if palm is not None:
                if frombone < numbones - 1:
                    length = palm.get_extension_length(t, self.c['fk_palm_local'])
                    last -= 1
                else:
                    return palm.get_bone_length()
        fk = self.c['fkpos']
        for i in range(frombone + 1, last):
            length += float(np.linalg.norm(fk[i + 1][3] - fk[i][3]))
        return length

    def get_tm_ik_target(self, t):
        tm = mm.ident()
        if not self.test(LIMBFLAG_LOCKED_IK):
            if self.iktarget is not None:
                tm = self.rig.obj_tm(self.iktarget, t)
                ipar = self.rig.get(self.rig.node_ctrl(self.iktarget))
                if isinstance(ipar, CATNodeControl):
                    tm = ipar.apply_child_offset(t, tm)
            elif self.layer_iktarget_trans is not None:
                tm, _ = self.layer_iktarget_trans.get_transformation(t, tm, tm, ONES, ONES.copy())
            else:
                tm = self.c['fk_target'].copy()
            tm = pre_rot(tm, mm.roty, math.pi)
        else:
            tm = self.c['ik_target'].copy()
        return tm

    def get_tm_palm_target(self, t, tm_fk_palm_local):
        numbones = self.num_bones
        ikpos = self.get_limb_ik_pos(t)
        last = int(math.floor(ikpos))
        tm_ik = self.get_tm_ik_target(t)
        palm = self.palm
        if palm is not None and last >= numbones:
            if last >= numbones + 1:
                tm_ik = palm.modify_ik_tm(t, tm_fk_palm_local, tm_ik)
            else:
                temp = palm.modify_ik_tm(t, tm_fk_palm_local, tm_ik.copy())
                tm_ik = blend_mat(tm_ik, temp, ikpos - last)
        return tm_ik

    def get_tm_fk_target(self, t, tm_ik_chain_parent):
        if tm_ik_chain_parent is not None:
            tm_parent = tm_ik_chain_parent
        else:
            b0 = self.bone(0)
            if b0 is None:
                return mm.ident()
            tm_parent = b0.get_parent_tm(t)
        if 'fk_target_local' in self.c:
            return mm.mul(self.c['fk_target_local'], tm_parent)
        self.c['fk_parent'] = tm_parent.copy()
        numbones = self.num_bones
        ikpos = self.get_limb_ik_pos(t)
        last = int(math.floor(ikpos))
        tm_world = tm_parent.copy()
        tm_last_bone_world = None
        tm_fk_palm_local = mm.ident()
        self.runtime_flags |= LIMBFLAG_LOCKED_FK
        try:
            palm = self.palm
            n_limb = numbones + (3 if palm is not None else 1)
            fk = [mm.ident() for _ in range(n_limb)]
            n_calc = numbones + (2 if palm is not None else 1)
            for i in range(n_calc):
                tm_world = self.get_bone_tm(i, t, tm_world)
                fk[i] = tm_world.copy()
                if i == numbones - 1:
                    tm_last_bone_world = tm_world.copy()
                elif i == numbones:
                    tm_fk_palm_local = mm.mul(tm_world, mm.inv(tm_last_bone_world))
                    if palm is not None:
                        fk[n_limb - 1] = palm.base_apply_child_offset(t, tm_world)
            d_off = ikpos - last
            if d_off != 0:
                d_off = max(d_off, 0.01)
                off = (fk[last + 1][3] - fk[last][3]) * d_off
                final = fk[last][3] + off
                fk[last + 1][3] = final
                tm_world = tm_world.copy()
                tm_world[3] = final
        finally:
            self.runtime_flags &= ~LIMBFLAG_LOCKED_FK
        self.c['fkpos'] = fk
        self.c['fk_target_local'] = mm.mul(tm_world, mm.inv(tm_parent))
        self.c['fk_palm_local'] = tm_fk_palm_local
        p3_fk_target = fk[last][3]
        for i in range(0, last - 2):
            b = self.bone(i)
            if b is None:
                continue
            child_lengths = self.get_limb_ik_length(t, i)
            bone_length = float(np.linalg.norm(fk[i + 1][3] - fk[i][3]))
            this_to_target = float(np.linalg.norm(fk[i][3] - p3_fk_target))
            fk_short = this_to_target - bone_length
            fk_long = min(child_lengths, this_to_target + bone_length)
            child_to_target = float(np.linalg.norm(fk[i + 1][3] - p3_fk_target))
            diff = fk_long - fk_short
            b.bend_ratio = (child_to_target - fk_short) / diff if diff > 0 else 0.0
        self.c['fk_target'] = tm_world.copy()
        return tm_world

    def get_bone_tm(self, i, t, tm_world):
        n = self.num_bones
        if 0 < i <= n:
            pb = self.bone(i - 1)
            if pb is not None:
                seg = pb.seg(0)
                for _ in range(1, pb.num_segs):
                    tm_world = seg.apply_child_offset(t, tm_world)
        if i >= n:
            return self.get_palm_tm(i, t, tm_world)
        b = self.bone(i)
        if b is not None:
            tm_world = b.get_value(t, tm_world)
        return tm_world

    def get_palm_tm(self, i, t, tm_world):
        n = self.num_bones
        palm = self.palm
        if palm is None:
            b = self.bone(n - 1)
            if b is not None:
                tm_world = b.seg(b.num_segs - 1).apply_child_offset(t, tm_world)
            return tm_world
        if i == n:
            return palm.get_value(t, tm_world)
        return palm.modify_fk_tm(t, tm_world)

    def get_ik_tms(self, t, p3_limb_root, tm_fk_parent, boneid=0):
        if self.test(LIMBFLAG_LOCKED_IK):
            return self.c['ik_target'], self.c['fk_target']
        valid = 'ik_target' in self.c
        self.get_tm_fk_target(t, tm_fk_parent if boneid == 0 else None)
        tm_palm_local = self.c['fk_palm_local']
        if not valid:
            self.c['limb_root'] = np.array(p3_limb_root, float)
            self.c['palm_local_vec'] = tm_palm_local[self.length_axis].copy()
            tm_ik = self.get_tm_palm_target(t, tm_palm_local)
            ratio = self.get_ikfk_ratio(t, -1)
            if ratio > 0:
                tm_fk = self.c['fk_target']
                root = np.array(p3_limb_root, float)
                to_ik = tm_ik[3] - root
                to_fk = tm_fk[3] - root
                d_ik = float(np.linalg.norm(to_ik))
                d_fk = float(np.linalg.norm(to_fk))
                to_ik = unit(to_ik)
                to_fk = unit(to_fk)
                axis = unit(np.cross(to_ik, to_fk))
                angle = math.acos(min(float(np.dot(to_ik, to_fk)), 1.0)) * ratio
                # RotAngleAxisMatrix: taken as the right-hand rule (IK direction turns to the FK direction).
                rot = angaxis_mat(axis, -angle)
                blended = mm.vtrans(to_ik, rot) * blend_float(d_ik, d_fk, ratio)
                tm_ik = blend_mat(tm_ik, tm_fk, ratio)
                tm_ik[3] = root + blended
            self.c['ik_target'] = tm_ik
        return self.c['ik_target'], self.c['fk_target']

    def is_limb_stretching(self, t, ik_target):
        return False    # no bone of the scenes has the flag CCFLAG_ANIM_STRETCHY

    def get_bone_fk_position(self, t, i):
        fk = self.c['fkpos']
        return fk[i][3] if i < len(fk) else np.zeros(3)

    def get_bone_fk_tm(self, t, i):
        return self.c['fkpos'][i].copy()


class BoneData(CATNodeControl):
    LAYERTRANS_REF = 1
    PB_LIMBDATA, PB_BONELENGTH, PB_NUMSEGS, PB_TWISTWEIGHT, PB_TWISTANGLE, PB_SEG_TAB = 0, 2, 7, 12, 13, 17

    @property
    def bend_ratio(self):
        return self.c.get('bend_ratio', 0.0)

    @bend_ratio.setter
    def bend_ratio(self, v):
        self.c['bend_ratio'] = v

    @property
    def evaluating(self):
        return self.c.get('evaluating', False)

    @evaluating.setter
    def evaluating(self, v):
        self.c['evaluating'] = v

    def link(self):
        super().link()
        pb = self.pb()
        self.limb = self.rig.get(self.pbref(self.PB_LIMBDATA))
        self.segs = [self.rig.get(i) for i in self.pbtab(self.PB_SEG_TAB)]
        if self.segs and self.segs[0] is not None:
            self.node = self.segs[0].node
        self.twist_ctrl = None
        tc = self.rig.std.pblock2_controller(self.ref(0), self.PB_TWISTWEIGHT) if hasattr(self.rig.std, 'pblock2_controller') else None
        if tc is not None:
            self.twist_ctrl = self.rig.get(tc)

    @property
    def num_segs(self):
        return len(self.segs)

    def seg(self, i):
        return self.segs[i] if 0 <= i < len(self.segs) else None

    def find_parent_cat_node_control(self):
        limb = self.limb
        if self.bone_id > 0:
            return limb.bone(self.bone_id - 1)
        return limb.collar if limb.collar is not None else limb.hub

    def get_child_cat_node_control(self):
        limb = self.limb
        if self.bone_id < limb.num_bones - 1:
            return limb.bone(self.bone_id + 1)
        if self.bone_id == limb.num_bones - 1 and limb.palm is not None:
            return limb.palm
        return None

    def calc_inheritance(self, t, tm):
        if self.bone_id == 0:
            tm = pre_rot(tm, mm.roty, math.pi)
        return self.base_calc_inheritance(t, tm)

    def calculate_twist_angle(self, tm_local):
        la = self.length_axis
        no_rot = np.zeros(3)
        no_rot[la] = 1.0
        tm_local = rotate_matrix_to_align_with_vector(mm.no_scale(tm_local), no_rot, la)
        tw = unit(tm_local[Y])
        d = float(np.dot(tw, (0.0, 1.0, 0.0)))
        if d <= -1 or d >= 1:
            return 0.0
        a = math.acos(d)
        other = tm_local[Z if la == X else X]
        if other[Y] > 0:
            a = -a
        return a

    def get_fk_value(self, t, tm_parent, p3_parent_scale, tm_value, p3_local_scale):
        limb = self.limb
        if 'local_fk' in self.c and self.test_cc(CNCFLAG_INHERIT_ANIM_ALL):
            loc = self.c['local_fk'].copy()
            loc[3] = loc[3] * p3_parent_scale
            tm_value = mm.mul(loc, tm_value)
            p3_local_scale = p3_local_scale * self.c['local_fk_scale']
            if limb.get_ikfk_ratio(t) >= 1.0:
                self.c['fktm'] = tm_value.copy()
        else:
            fktm, ls = self.base_calc_world_transform(t, tm_parent, p3_parent_scale, tm_value.copy(), ONES.copy())
            self.c['fktm'] = fktm.copy()
            self.c['local_fk_scale'] = ls
            self.c['local_fk'] = mm.mul(fktm, mm.inv(tm_value))
            tm_value = fktm
            p3_local_scale = p3_local_scale * ls
        return tm_value, p3_local_scale

    def get_ik_value(self, t, p3_curr_pos, tm_parent, tm_value, p3_local_scale):
        limb = self.limb
        bid = self.bone_id
        la = self.length_axis
        tm_ik_target, tm_fk_target = limb.get_ik_tms(t, p3_curr_pos, tm_parent, bid)
        child_lengths = limb.get_limb_ik_length(t, bid)
        numbones = limb.num_bones
        numbones = min(int(math.floor(limb.get_limb_ik_pos(t))) + 1, numbones)
        fktm = self.c['fktm']
        if bid == 0:
            iktm = tm_value.copy()
        else:
            iktm = mm.no_scale(limb.get_bone_fk_tm(t, bid))
        bonepos = tm_value[3].copy()
        ik_vec = tm_ik_target[3] - bonepos
        fk_vec = tm_fk_target[3] - iktm[3]
        ik_len = float(np.linalg.norm(ik_vec))
        if ik_len > 0:
            ik_vec = ik_vec / ik_len
        if bid == 0:
            fk_vec0 = tm_fk_target[3] - bonepos
            fl = float(np.linalg.norm(fk_vec0))
            if fl > 0:
                fk_vec0 = fk_vec0 / fl
            ang = math.acos(cos_limit(float(np.dot(ik_vec, fk_vec0))))
            axis = unit(np.cross(ik_vec, fk_vec0))
            iktm = rotate_matrix(iktm, axis, ang)
            limb.c['ik_offset_ax'] = (axis, ang)
            if limb.upnode is not None:
                up_tm = self.rig.node_tm(limb.upnode, t)
                to_up = up_tm[3] - fktm[3]
                vec1 = unit(np.cross(to_up, ik_vec))
                vec2 = unit(np.cross(iktm[la], ik_vec))
                up_ang = math.acos(cos_limit(float(np.dot(vec1, vec2))))
                up_ang *= 1.0 - limb.get_ikfk_ratio(t, bid)
                up_axis = unit(np.cross(vec1, vec2))
                iktm = rotate_matrix(iktm, up_axis, up_ang)
                limb.c['upnode_offset_ax'] = (up_axis, up_ang)
            iktm[3] = bonepos
        else:
            axis, ang = limb.c['ik_offset_ax']
            iktm = rotate_matrix(iktm, axis, ang)
            if limb.upnode is not None:
                up_axis, up_ang = limb.c['upnode_offset_ax']
                iktm = rotate_matrix(iktm, up_axis, up_ang)
            fk_vec = unit(fk_vec)
            a = -math.acos(cos_limit(float(np.dot(fktm[la], fk_vec))))
            ax = unit(np.cross(fktm[la], fk_vec))
            fk_look = rotate_matrix(fktm, ax, a)
            fk_look[3] = fktm[3]
            a = -math.acos(cos_limit(float(np.dot(iktm[la], ik_vec))))
            ax = unit(np.cross(iktm[la], ik_vec))
            ik_look = rotate_matrix(iktm, ax, a)
            ik_look[3] = iktm[3]
            fk_delta = mm.mul(fktm, mm.inv(fk_look))
            iktm = mm.mul(fk_delta, ik_look)
            iktm[3] = bonepos
        bone_length = float(np.linalg.norm(limb.get_bone_fk_position(t, bid) - limb.get_bone_fk_position(t, bid + 1)))
        this_ik_bone_vec = iktm[la].copy()
        cos_angle = cos_limit(float(np.dot(ik_vec, this_ik_bone_vec)))
        ax_angle = -math.acos(cos_angle)
        ax_axis = unit(np.cross(this_ik_bone_vec, ik_vec))
        palm = limb.palm
        if (ik_len >= (child_lengths + bone_length) or bone_length > (ik_len + child_lengths)
                or (bid == numbones - 1 and palm is None)):
            iktm = rotate_matrix(iktm, ax_axis, ax_angle)
            iktm[3] = bonepos
        else:
            if not (bid == numbones - 1 and palm is not None):
                if bid <= numbones - 2 or (bid <= numbones - 1 and palm is not None):
                    if bid == numbones - 2 or (bid == numbones - 1 and palm is not None):
                        imag = child_lengths
                    else:
                        ik_short = ik_len - bone_length
                        ik_long = min(child_lengths, ik_len + bone_length)
                        imag = ik_short + (ik_long - ik_short) * self.bend_ratio
                    if imag > bone_length + ik_len:
                        ax_angle += math.pi
                    else:
                        cd = (bone_length ** 2 + ik_len ** 2 - imag ** 2) / (2 * bone_length * ik_len)
                        desired = -math.acos(cos_limit(cd))
                        ax_angle = -(desired - ax_angle)
                iktm = rotate_matrix(iktm, ax_axis, ax_angle)
                iktm[3] = bonepos
            else:
                target_align = palm.get_target_align(t)
                imag = palm.get_bone_length()
                iktm = rotate_matrix(iktm, ax_axis, ax_angle)
                iktm[3] = bonepos
                tm_target_align = iktm.copy()
                if target_align < 1.0:
                    ca = (bone_length ** 2 + ik_len ** 2 - imag ** 2) / (2 * bone_length * ik_len)
                    a = math.acos(cos_limit(ca))
                    limb.get_tm_fk_target(t, None)
                    tm_palm_local = limb.c['fk_palm_local']
                    ax = unit(np.cross(iktm[la], mm.mul(tm_palm_local, iktm)[la]))
                    iktm = rotate_matrix(iktm, ax, a)
                    iktm = blend_rot(iktm, tm_target_align, target_align)
                iktm[3] = bonepos
        if bid == 0 and limb.is_leg and limb.test(LIMBFLAG_FFB_WORLDZ):
            if limb.get_force_feedback(t) > 0.0:
                raise NotImplementedError('leg force feedback')
        self.c['iktm'] = iktm.copy()
        return iktm, p3_local_scale

    def calc_world_transform(self, t, tm_orig_parent, p3_parent_scale, tm_value, p3_local_scale):
        limb = self.limb
        if limb is None:
            raise CatError('limb bone with no limb')
        bid = self.bone_id
        ratio = limb.get_ikfk_ratio(t, bid)
        tm_parent = tm_value.copy()
        tm_value, p3_local_scale = self.get_fk_value(t, tm_parent, p3_parent_scale, tm_value, p3_local_scale)
        if ratio < 1.0:
            tm_fk_value = tm_value.copy()
            tm_value, p3_local_scale = self.get_ik_value(t, tm_parent[3].copy(), tm_orig_parent, tm_value, p3_local_scale)
            ikpos = limb.get_limb_ik_pos(t)
            if bid < ikpos < bid + 1:
                tm_value = blend_rot(tm_value, tm_fk_value, 1.0 - (ikpos - bid))
        if self.evaluating:
            raise CatError('BoneData evaluates two times')
        self.evaluating = True
        try:
            if self.num_segs > 1:
                if bid == 0:
                    fixed = pre_rot(tm_orig_parent, mm.roty, math.pi / 2 * limb.lmr)
                    self.c['parent_twist'] = self.calculate_twist_angle(mm.mul(tm_value, mm.inv(mm.no_scale(fixed))))
                else:
                    self.c['parent_twist'] = self.calculate_twist_angle(mm.mul(tm_value, mm.inv(mm.no_scale(tm_orig_parent))))
                child = self.get_child_cat_node_control()
                if child is not None:
                    # As in the C++ code, this evaluation leaves its results in the caches of the
                    # child. The evaluation of the node of the child replaces them later.
                    tm_child = child.get_value(t, tm_value.copy())
                    tw_parent = tm_value
                    if isinstance(child, PalmTrans2):
                        tw_parent = self.fix_twist_for_palm(tm_value, limb)
                    self.c['child_twist'] = self.calculate_twist_angle(mm.mul(mm.no_scale(tm_child), mm.inv(tw_parent)))
                else:
                    self.c['child_twist'] = 0.0
            else:
                self.c['child_twist'] = self.c['parent_twist'] = 0.0
        finally:
            self.evaluating = False
        return tm_value, p3_local_scale

    def fix_twist_for_palm(self, tm, limb):
        res = tm.copy()
        if limb is not None and not limb.is_leg:
            if self.length_axis == Z:
                if limb.lmr == -1:
                    res[Y] = tm[X]
                    res[X] = -tm[Y]
                else:
                    res[Y] = -tm[X]
                    res[X] = tm[Y]
            else:
                if limb.lmr == -1:
                    res[Y] = -tm[Z]
                    res[Z] = tm[Y]
                else:
                    res[Y] = tm[Z]
                    res[Z] = -tm[Y]
        return res

    def get_twist_weight(self, ratio):
        if self.twist_ctrl is not None:
            return self.twist_ctrl.value(ratio)
        return ratio

    def get_twist_angle(self, seg_id):
        ratio = float(seg_id) / self.num_segs
        w = self.get_twist_weight(ratio)
        w0 = self.get_twist_weight(0.0)
        w1 = self.get_twist_weight(1.0)
        w = (w - w0) / (w1 - w0) if w1 != w0 else 0.0
        from_parent = self.c.get('parent_twist', 0.0) * w0 if w0 < 0 else 0.0
        to_child = self.c.get('child_twist', 0.0) * w1 if w1 > 0 else 0.0
        return blend_float(from_parent, to_child, w)


class BoneSegTrans(CATNodeControl):
    PB_BONEDATA = 0

    def bone_data(self):
        return self.rig.get(self.pbref(self.PB_BONEDATA))

    def find_parent_cat_node_control(self):
        b = self.bone_data()
        if self.bone_id > 0:
            return b.seg(self.bone_id - 1)
        bp = b.get_parent_cat_node_control(True)
        if isinstance(bp, BoneData):
            return bp.seg(bp.num_segs - 1)
        return bp

    def get_value(self, t, val):
        b = self.bone_data()
        if b is None:
            raise CatError('bone segment with no bone data')
        seg = self.bone_id
        if seg == 0:
            tm = b.get_value(t, val)
        else:
            tm = self.base_apply_child_offset(t, val)
        tw = b.get_twist_angle(seg)
        if seg != 0:
            tw -= b.get_twist_angle(seg - 1)
        if self.length_axis == X:
            tm = pre_rot(tm, mm.rotx, tw)
        else:
            tm = pre_rot(tm, mm.rotz, -tw)
        self.c['world'] = mm.no_scale(tm)
        return tm

    def apply_child_offset(self, t, tm):
        tm = self.base_apply_child_offset(t, tm)
        b = self.bone_data()
        if b is None:
            return tm
        if self.bone_id == b.num_segs - 1:
            if not b.evaluating:
                pos = tm[3].copy()
                tm = b.tm_bone_world(t).copy()
                s = b.world_scale(t)
                if not np.array_equal(s, ONES):
                    tm = mm.pre_scale(tm, s)
                tm[3] = pos
        return tm


class PalmTrans2(CATNodeControl):
    LAYERTRANS_REF = 2
    PB_LIMBDATA, PB_LAYERTARGETALIGN, PB_NUMDIGITS, PB_DIGITDATATAB = 0, 7, 10, 11

    def link(self):
        super().link()
        pb = self.pb()
        self.limb = self.rig.get(self.pbref(self.PB_LIMBDATA))
        self.ikpivotoffset = np.zeros(3)
        std = self.rig.std
        tc = std.pblock2_controller(self.ref(0), self.PB_LAYERTARGETALIGN) if hasattr(std, 'pblock2_controller') else None
        self.target_align_ctrl = self.rig.get(tc) if tc is not None else None
        self.target_align_idx = tc

    def apply_child_offset(self, t, tm):
        return tm       # PalmTrans2.h: an empty override

    def find_parent_cat_node_control(self):
        limb = self.limb
        if limb is not None and limb.num_bones > 0:
            b = limb.bone(limb.num_bones - 1)
            if b is not None and b.num_segs > 0:
                return b.seg(b.num_segs - 1)
        return None

    def get_target_align(self, t):
        limb = self.limb
        if limb is None:
            return 0.0
        if limb.test(LIMBFLAG_LOCKED_IK):
            return 1.0
        if self.target_align_ctrl is not None:
            v = self.target_align_ctrl.get_value(t)
        elif self.target_align_idx is not None:
            v = self.rig.std.float_value(self.target_align_idx, t)
        else:
            v = float(self.pb().get(self.PB_LAYERTARGETALIGN, 0.0))
        return min(1.0, max(v, 0.0))

    def get_fk_value(self, t, tm_parent, tm_world, p3_local_scale):
        return self.base_calc_world_transform(t, tm_parent, ONES, tm_world, p3_local_scale)

    def get_ik_value(self, t, p3_palm_target, tm_fk_world, tm_world):
        ankle = tm_world[3].copy()
        la = self.length_axis
        target_align = self.get_target_align(t)
        limb = self.limb
        if limb is not None:
            target_align *= 1.0 - limb.get_ikfk_ratio(t)
        tm_ik_world = mm.no_scale(self.c['palm_target'])
        tm_ik_world = blend_rot(tm_ik_world, tm_fk_world, 1.0 - target_align)
        zv = unit(p3_palm_target - tm_world[3])
        ang = math.acos(min(float(np.dot(zv, unit(tm_ik_world[la]))), 1.0))
        axis = unit(np.cross(zv, tm_ik_world[la]))
        tm_ik_world = rotate_matrix(tm_ik_world, axis, ang)
        tm_ik_world[3] = ankle
        return tm_ik_world

    def calc_world_transform(self, t, tm_parent, p3_parent_scale, tm_world, p3_local_scale):
        limb = self.limb
        if limb is None:
            raise CatError('palm with no limb')
        n = limb.num_bones
        ratio = limb.get_ikfk_ratio(t, n)
        cur = tm_world[3].copy()
        if limb.test(LIMBFLAG_LOCKED_IK):
            raise NotImplementedError('locked IK')
        tm_world, p3_local_scale = self.get_fk_value(t, tm_parent, tm_world, p3_local_scale)
        tm_world[3] = cur
        if ratio < 1.0:
            tm_fk = tm_world.copy()
            p3_ik = self.c['palm_target'][3].copy()
            if ratio > 0:
                p3_ik = p3_ik + (limb.get_bone_fk_position(t, n + 2) - p3_ik) * ratio
            tm_world = self.get_ik_value(t, p3_ik, tm_fk, tm_world)
            ikpos = limb.get_limb_ik_pos(t)
            if n < ikpos < n + 1:
                tm_world = blend_rot(tm_world, tm_fk, 1.0 - (ikpos - n))
        return tm_world, p3_local_scale

    def modify_fk_tm(self, t, tm_fk_target):
        child = self.base_apply_child_offset(t, tm_fk_target)
        return blend_mat(tm_fk_target, child, 1.0 - self.get_target_align(t))

    def calc_palm_rel_to_ik_target(self, t, tm_ik_target):
        lt = self.layer_trans
        tm, _ = lt.get_transformation(t, tm_ik_target, tm_ik_target.copy(), ONES, ONES.copy())
        return mm.mul(tm, mm.inv(tm_ik_target))

    def modify_ik_tm(self, t, tm_fk_local, tm_ik_target):
        limb = self.limb
        target_align = self.get_target_align(t)
        tm_palm_local = tm_fk_local.copy()
        rel = self.calc_palm_rel_to_ik_target(t, tm_ik_target)
        ik_pos = rel[3].copy()
        tm_palm_local = blend_mat(tm_palm_local, rel, target_align)
        tm_palm_local[3] = ik_pos
        palmlength = self.get_bone_length()
        shift = palmlength * target_align
        temp = tm_ik_target.copy()
        if limb.is_leg:
            temp = mm.mul(tm_palm_local, temp)
        palm_target = mm.pre_translate(temp, self.ikpivotoffset * self.catunits)
        self.c['palm_target'] = palm_target
        off = np.zeros(3)
        off[self.length_axis] = -shift
        temp = mm.pre_translate(temp, off)
        r = tm_ik_target.copy()
        r[3] = temp[3]
        return r

    def get_extension_length(self, t, tm_palm_fk_local):
        limb = self.limb
        if limb is None:
            return 0.0
        target_align = self.get_target_align(t)
        if target_align < 1.0:
            palmlength = self.get_bone_length()
            la = self.length_axis
            ang = math.acos(min(1.0, max(-1.0, float(tm_palm_fk_local[la][la]))))
            last = limb.bone(limb.num_bones - 1).get_bone_length()
            extra = math.sqrt(palmlength ** 2 + last ** 2 - 2.0 * palmlength * last * math.cos(math.pi - ang)) - last
            return extra * (1.0 - target_align)
        return 0.0


class DigitData(CATControl):
    PB_ROOTPOS, PB_PALM, PB_NUMBONES, PB_SEGTRANSTAB = 5, 6, 7, 8

    def root_pos(self):
        v = self.pb().get(self.PB_ROOTPOS)
        return np.asarray(v, float) if v is not None else np.zeros(3)


class DigitSegTrans(CATNodeControl):
    LAYERTRANS_REF = 1
    PB_DIGITDATA = 0

    def digit(self):
        return self.rig.get(self.pbref(self.PB_DIGITDATA))

    def calc_parent_transform(self, t, tm):
        tm, ps = self.base_calc_parent_transform(t, tm)
        if self.bone_id == 0:
            p = self.get_parent_cat_node_control()
            if p is not None:
                d = self.digit()
                if d is not None:
                    tm = mm.pre_translate(tm, p.get_bone_dimensions(t) * p.world_scale(t) * d.root_pos())
        return tm, ps


class ArbBoneTrans(CATNodeControl):
    LAYERTRANS_REF = 0

    def calc_parent_transform(self, t, tm):
        p = self.get_parent_cat_node_control()
        if p is not None:
            if isinstance(p, BoneSegTrans):
                tm = p.base_apply_child_offset(t, tm)
            else:
                tm = p.apply_child_offset(t, tm)
        return self.calc_inheritance(t, tm)


class IKTargTrans(CATNodeControl):
    LAYERTRANS_REF = 0

    def apply_child_offset(self, t, tm):
        return tm       # IKTargController.h: an empty override


class FootTrans2(CATNodeControl):
    LAYERTRANS_REF = 3


class GizmoTransform(Base):
    def get_value(self, t, val):
        target = self.ref(0)
        if target >= 0:
            return self.rig.node_tm(target, t).copy()
        return val
