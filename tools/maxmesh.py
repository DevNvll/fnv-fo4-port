"""Extract mesh, skin and material data from the streams of a 3ds Max scene file.

Module use:
    from maxmesh import extract
    data = extract('ole/reload_48')        # a dict, see tools/maxmesh-notes.md

Command use:
    python3 tools/maxmesh.py ole/SCENE -o out/mesh/SCENE

The command writes out/mesh/SCENE/meshes.json. All matrices have the shape
(4, 3) of tools/maxmath.py: rows 0 to 2 are the axes, row 3 is the translation,
and a point is a row vector (p' = p @ m[:3] + m[3]).
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maxchunks import load_stream  # noqa: E402
from maxscene import Scene  # noqa: E402
import maxmath as mm  # noqa: E402

DERIVED_OBJECT = 0x2032          # chunk id of a derived object (not in the class directory)
CLASS_EDITABLE_MESH = (0xE44F10B3, 0)
CLASS_SKIN = (0x0095C723, 0x00015666)
CLASS_DISMEMBER = (0xE9A0A68E, 0xB091BD48)
CLASS_STANDARD_MTL = (0x2, 0)
CLASS_MULTI_MTL = (0x200, 0)
CLASS_BITMAP_TEX = (0x240, 0)
CLASS_NORMAL_BUMP = (0x243E22C6, 0x63F6A014)
SUPER_MATERIAL = 0xC00
SUPER_TEXMAP = 0xC10

# Slot names of the Texmaps object. The first table is for the standard shaders
# (Blinn and others). The second table is for the "Niftools Shader" of the NifTools
# plug-in. The file does not store the slot names of that shader: slot 0 has the
# diffuse texture and slot 5 has the normal map in each material of the MP7 scenes,
# the other slots have no name here.
STD_SLOTS = ['ambient', 'diffuse', 'specular', 'specular_level', 'glossiness',
             'self_illumination', 'opacity', 'filter', 'bump', 'reflection',
             'refraction', 'displacement']
NIF_SLOTS = {0: 'base', 5: 'bump'}


# ---------------------------------------------------------------- small readers

def _utf16(data):
    return data.decode('utf-16-le').rstrip('\x00')


def read_matrix3(chunk):
    """A Matrix3 as Matrix3::Save writes it: 0x03E8 has 12 floats, 0x03F2 has the flags."""
    m = np.frombuffer(chunk.find(0x03E8).data, '<f4', 12).reshape(4, 3).astype(np.float64)
    fc = chunk.find(0x03F2)
    flags = struct.unpack('<I', fc.data)[0] if fc is not None else 0
    return m, flags


def read_bitarray(data):
    """A BitArray: bit count, then the bits (least significant bit first)."""
    n = struct.unpack_from('<I', data, 0)[0]
    bits = np.unpackbits(np.frombuffer(data, np.uint8, (n + 7) // 8, 4), bitorder='little')[:n]
    return bits.astype(bool)


def read_assets(folder):
    """FileAssetMetaData3: asset id (16 bytes) -> (asset type, file path)."""
    path = os.path.join(folder, 'FileAssetMetaData3')
    out = {}
    if not os.path.exists(path):
        return out
    d = load_stream(path)
    pos = 0
    while pos + 16 <= len(d):
        guid = d[pos:pos + 16].hex()
        pos += 16
        fields = []
        for _ in range(3):
            n = struct.unpack_from('<I', d, pos)[0]
            pos += 4
            fields.append(d[pos:pos + n * 2].decode('utf-16-le'))
            pos += n * 2 + 2
        out[guid] = (fields[0], fields[1])
    return out


def classid(o):
    return o.cls['classid'] if o.cls else None


def superid(o):
    return o.cls['super'] if o.cls else None


# ---------------------------------------------------------------- the node

def node_offset(node):
    """Object offset transform of a node: chunks 0x096A, 0x096B and 0x096C."""
    pos = np.zeros(3)
    rot = np.array([0.0, 0.0, 0.0, 1.0])
    scale = np.ones(3)
    axis = np.array([0.0, 0.0, 0.0, 1.0])
    c = node.chunk.find(0x096A)
    if c is not None:
        pos = np.array(struct.unpack('<3f', c.data[:12]), float)
    c = node.chunk.find(0x096B)
    if c is not None:
        rot = np.array(struct.unpack('<4f', c.data[:16]), float)
    c = node.chunk.find(0x096C)
    if c is not None:
        v = struct.unpack('<7f', c.data[:28])
        scale = np.array(v[:3], float)
        axis = np.array(v[3:], float)
    return pos, rot, scale, axis


def offset_matrix(pos, rot, scale, axis):
    """Matrix of the object offset as INode::GetObjectTM applies it.

    objectTM = scale * rotation * translation * nodeTM. The scale is
    inverse(axis) * diag(scale) * axis (class ScaleValue).
    """
    m = mm.ident()
    if not (np.allclose(scale, 1.0) and np.allclose(axis[:3], 0.0)):
        u = mm.quat_to_mat(axis)
        m = mm.mul(mm.mul(mm.inv(u), mm.scale_mat(scale)), u)
    m = mm.mul(m, mm.quat_to_mat(rot))
    m = mm.mul(m, mm.trans_mat(pos))
    return m


def _bezier_float_static(o):
    c = o.chunk.find(0x7127)
    v = c.find(0x2501) if c is not None else o.chunk.find(0x2501)
    if v is None:
        return None
    return struct.unpack('<f', v.data[:4])[0]


def _xyz_static(scene, o):
    vals = []
    for r in o.refs[:3]:
        if r < 0:
            return None
        v = _bezier_float_static(scene.objs[r])
        if v is None:
            return None
        vals.append(v)
    return np.array(vals, float)


def prs_static(scene, ctl):
    """Static value of a Position/Rotation/Scale controller (the value at the save).

    Returns (matrix, parts) or (None, reason). Only Position XYZ, Euler XYZ and
    Bezier Scale with static values are read.
    """
    if ctl.cname != 'Position/Rotation/Scale':
        return None, 'controller %s' % ctl.cname
    parts = {}
    refs = ctl.refs + [-1] * 3
    p = scene.objs[refs[0]] if refs[0] >= 0 else None
    r = scene.objs[refs[1]] if refs[1] >= 0 else None
    s = scene.objs[refs[2]] if refs[2] >= 0 else None
    if p is None or p.cname != 'Position XYZ':
        return None, 'position controller %s' % (p.cname if p else None)
    pos = _xyz_static(scene, p)
    if r is None or r.cname != 'Euler XYZ':
        return None, 'rotation controller %s' % (r.cname if r else None)
    order_chunk = r.chunk.find(0x1003)
    order = struct.unpack('<i', order_chunk.data[:4])[0] if order_chunk is not None else 0
    eul = _xyz_static(scene, r)
    if pos is None or eul is None:
        return None, 'no static value'
    if order != 0:
        return None, 'euler order %d' % order
    scale = np.ones(3)
    axis = np.array([0.0, 0.0, 0.0, 1.0])
    if s is not None:
        c = s.chunk.find(0x2505)
        if c is not None:
            v = struct.unpack('<7f', c.data[:28])
            scale = np.array(v[:3], float)
            axis = np.array(v[3:], float)
        elif s.cname == 'ScaleXYZ':
            sv = _xyz_static(scene, s)
            if sv is not None:
                scale = sv
    # PRS: the matrix is scale, then rotation, then translation (relative to the parent).
    m = mm.ident()
    if not np.allclose(scale, 1.0, atol=1e-4):
        u = mm.quat_to_mat(axis)
        m = mm.mul(mm.mul(mm.inv(u), mm.scale_mat(scale)), u)
    m = mm.mul(m, mm.euler_to_mat(eul))
    m = mm.mul(m, mm.trans_mat(pos))
    parts = {'pos': pos.tolist(), 'euler_xyz_rad': eul.tolist(), 'scale': scale.tolist(),
             'scale_axis': axis.tolist()}
    return m, parts


def node_static(scene, node):
    """Static local transform of a node, as far as the file gives it.

    A Link Constraint has a target node and a Position/Rotation/Scale controller;
    with one target the node transform is prs * nodeTM(target).
    """
    ctl = scene.objs[node.refs[0]] if node.refs and node.refs[0] >= 0 else None
    if ctl is None:
        return {'controller': None}
    out = {'controller': ctl.cname}
    if ctl.cname == 'Position/Rotation/Scale':
        m, parts = prs_static(scene, ctl)
        if m is None:
            out['static'] = False
            out['reason'] = parts
        else:
            out['static'] = True
            out.update(parts)
            out['matrix'] = m.tolist()
            out['relative_to'] = 'parent'
    elif ctl.cname == 'Link Constraint':
        targets = []
        if len(ctl.refs) > 2 and ctl.refs[2] >= 0:
            pb = scene.objs[ctl.refs[2]]
            targets = [scene.objs[r].name for r in (pb.refs or []) if r >= 0]
        out['link_targets'] = targets
        sub = scene.objs[ctl.refs[0]] if ctl.refs and ctl.refs[0] >= 0 else None
        m, parts = prs_static(scene, sub) if sub is not None else (None, 'no controller')
        if m is None:
            out['static'] = False
            out['reason'] = parts
        else:
            out['static'] = len(targets) == 1
            out.update(parts)
            out['matrix'] = m.tolist()
            out['relative_to'] = 'link target %s' % (targets[0] if targets else None)
    else:
        out['static'] = False
    return out


def rest_to(scene, node, ancestor_name):
    """Rest matrix of a node relative to the node `ancestor_name`, from static data only.

    Each step goes to the parent (Position/Rotation/Scale) or to the link target
    (Link Constraint with one target). Returns (matrix or None, list of steps).
    """
    m = mm.ident()
    steps = []
    cur = node
    for _ in range(64):
        if cur.name == ancestor_name:
            return m, steps
        st = node_static(scene, cur)
        if not st.get('static'):
            steps.append('%s: %s is not static' % (cur.name, st.get('controller')))
            return None, steps
        m = mm.mul(m, np.array(st['matrix']))
        if st['controller'] == 'Link Constraint':
            tname = st['link_targets'][0]
            nxt = [n for n in scene.nodes() if n.name == tname]
            steps.append('%s -> link target %s' % (cur.name, tname))
            if not nxt:
                return None, steps
            cur = nxt[0]
        else:
            par = scene.objs[cur.parent] if cur.parent is not None and 0 <= cur.parent < len(scene.objs) else None
            if par is None or par.name is None:
                steps.append('%s -> scene root' % cur.name)
                return None, steps
            steps.append('%s -> parent %s' % (cur.name, par.name))
            cur = par
    return None, steps


def rest_between(scene, node, ref_name):
    """Rest matrix of `node` relative to the node `ref_name`.

    The two chains can meet at a common ancestor that is not `ref_name` (the node
    "MP7 Parts" is linked to "Weapon", the parent of "MP7").
    """
    m, steps = rest_to(scene, node, ref_name)
    if m is not None:
        return m, steps
    refs = [n for n in scene.nodes() if n.name == ref_name]
    if not refs:
        return None, steps
    ref = refs[0]
    # Walk the static chain of the reference node and try each ancestor as the meeting point.
    cur = ref
    ref_m = mm.ident()
    ref_steps = []
    for _ in range(64):
        st = node_static(scene, cur)
        if not st.get('static') or st['controller'] != 'Position/Rotation/Scale':
            break
        ref_m = mm.mul(ref_m, np.array(st['matrix']))
        par = scene.objs[cur.parent]
        if par.name is None:
            break
        ref_steps.append('%s -> parent %s' % (cur.name, par.name))
        m2, steps2 = rest_to(scene, node, par.name)
        if m2 is not None:
            return mm.mul(m2, mm.inv(ref_m)), steps2 + ['inverse of: ' + ', '.join(ref_steps)]
        cur = par
    return None, steps


# ---------------------------------------------------------------- the mesh

def read_mesh(obj):
    """Geometry of an Editable Mesh object (chunk 0x08FE)."""
    m = obj.chunk.find(0x08FE)
    if m is None:
        raise ValueError('object %d has no mesh chunk 0x08FE' % obj.index)
    out = {'maps': {}}
    channel = None
    for c in m.children:
        if c.id == 0x0914:
            n = struct.unpack_from('<I', c.data, 0)[0]
            out['verts'] = np.frombuffer(c.data, '<f4', n * 3, 4).reshape(n, 3).copy()
        elif c.id == 0x0912:
            n = struct.unpack_from('<I', c.data, 0)[0]
            rec = np.frombuffer(c.data, '<u4', n * 5, 4).reshape(n, 5)
            out['faces'] = rec[:, :3].astype(np.int64)
            out['smoothing_groups'] = rec[:, 3].copy()
            out['face_flags'] = (rec[:, 4] & 0xFFFF).astype(np.int64)
            out['material_ids'] = (rec[:, 4] >> 16).astype(np.int64)
        elif c.id == 0x0959:
            channel = struct.unpack('<i', c.data[:4])[0]
        elif c.id == 0x2394:
            n = struct.unpack_from('<I', c.data, 0)[0]
            tv = np.frombuffer(c.data, '<f4', n * 3, 4).reshape(n, 3).copy()
            out['maps'].setdefault(channel, {})['verts'] = tv
        elif c.id == 0x2396:
            n = struct.unpack_from('<I', c.data, 0)[0]
            tf = np.frombuffer(c.data, '<u4', n * 3, 4).reshape(n, 3).astype(np.int64)
            out['maps'].setdefault(channel, {})['faces'] = tf
        elif c.id == 0x23A0:
            out['normal_spec'] = read_normal_spec(c)
    if 'verts' not in out or 'faces' not in out:
        raise ValueError('object %d has no vertex or face chunk' % obj.index)
    nv = len(out['verts'])
    if len(out['faces']) and out['faces'].max() >= nv:
        raise ValueError('object %d: a face index is out of range' % obj.index)
    for ch, mp in out['maps'].items():
        if 'verts' in mp and 'faces' in mp and len(mp['faces']):
            if len(mp['faces']) != len(out['faces']) or mp['faces'].max() >= len(mp['verts']):
                raise ValueError('object %d: map channel %s does not fit the faces' % (obj.index, ch))
    return out


def read_normal_spec(c):
    """MeshNormalSpec (chunk 0x23A0): the normals that the scene file stores."""
    out = {}
    cur = None
    faces = None
    spec = None
    for ch in c.children:
        if ch.id == 0x0100:
            out['flags'] = struct.unpack('<I', ch.data[:4])[0]
        elif ch.id == 0x0110:
            n = struct.unpack_from('<I', ch.data, 0)[0]
            out['normals'] = np.frombuffer(ch.data, '<f4', n * 3, 4).reshape(n, 3).copy()
        elif ch.id == 0x0114:
            b = ch.find(0x2700)
            if b is not None:
                out['explicit'] = read_bitarray(b.data)
        elif ch.id == 0x0120:
            n = struct.unpack('<I', ch.data[:4])[0]
            faces = np.full((n, 3), -1, np.int64)
            spec = np.zeros(n, np.int64)
        elif ch.id == 0x0124:
            cur = struct.unpack('<I', ch.data[:4])[0]
        elif ch.id == 0x0128 and faces is not None and cur is not None:
            ids = ch.find(0x0200)
            fl = ch.find(0x0210)
            if ids is not None:
                faces[cur] = struct.unpack('<3i', ids.data[:12])
            if fl is not None:
                spec[cur] = fl.data[0] & 7      # the other 3 bytes are not initialized
    if faces is not None:
        out['faces'] = faces
        out['specified'] = spec
    return out


def smooth_normals(verts, faces, smg):
    """Normals from the smoothing groups, one for each face corner: shape (faces, 3, 3).

    At a vertex, two faces share a normal when they share a smoothing group bit,
    directly or through a chain of faces at that vertex. The normal is the sum of
    the face normals, each with the angle of the face at the vertex as its weight.
    A face with smoothing group 0 keeps its face normal.
    """
    v = verts.astype(np.float64)
    f = faces
    nf = len(f)
    p = v[f]                                            # (nf, 3, 3)
    fn = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    ln = np.linalg.norm(fn, axis=1)
    fnu = np.where(ln[:, None] > 0, fn / np.maximum(ln, 1e-30)[:, None], 0.0)
    ang = np.zeros((nf, 3))
    for c in range(3):
        a = p[:, (c + 1) % 3] - p[:, c]
        b = p[:, (c + 2) % 3] - p[:, c]
        na = np.linalg.norm(a, axis=1)
        nb = np.linalg.norm(b, axis=1)
        cosv = np.einsum('ij,ij->i', a, b) / np.maximum(na * nb, 1e-30)
        ang[:, c] = np.arccos(np.clip(cosv, -1.0, 1.0))
    out = np.repeat(fnu[:, None, :], 3, axis=1).copy()
    by_vert = {}
    for fi in range(nf):
        for c in range(3):
            by_vert.setdefault(int(f[fi, c]), []).append((fi, c))
    for vi, corners in by_vert.items():
        n = len(corners)
        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        groups = [int(smg[fi]) for fi, _ in corners]
        for i in range(n):
            if groups[i] == 0:
                continue
            for j in range(i + 1, n):
                if groups[i] & groups[j]:
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[rj] = ri
        sums = {}
        for i, (fi, c) in enumerate(corners):
            r = find(i)
            sums[r] = sums.get(r, 0.0) + fnu[fi] * ang[fi, c]
        for i, (fi, c) in enumerate(corners):
            s = sums[find(i)]
            l2 = np.linalg.norm(s)
            if l2 > 1e-12:
                out[fi, c] = s / l2
    return out


def spec_corner_normals(mesh):
    """Stored normals for each face corner (faces, 3, 3), or None when the mesh has none."""
    ns = mesh.get('normal_spec')
    if not ns or 'faces' not in ns or 'normals' not in ns:
        return None
    ids = ns['faces']
    if len(ids) != len(mesh['faces']) or ids.min() < 0 or ids.max() >= len(ns['normals']):
        return None
    return ns['normals'][ids].astype(np.float64)


# ---------------------------------------------------------------- object stack

def resolve_object(scene, idx):
    """Follow derived objects. Returns (base object, [(modifier object, context chunk)]).

    A derived object (chunk id 0x2032) references its modifiers (the top of the
    stack first) and then its base object. It has one container 0x2500 for each
    modifier, in the same order.
    """
    mods = []
    o = scene.objs[idx]
    guard = 0
    while o.chunk.id == DERIVED_OBJECT and guard < 16:
        guard += 1
        apps = o.chunk.find_all(0x2500)
        refs = o.refs
        for i, r in enumerate(refs[:-1]):
            if r >= 0:
                mods.append((scene.objs[r], apps[i] if i < len(apps) else None))
        o = scene.objs[refs[-1]]
    return o, mods


# ---------------------------------------------------------------- the skin

def _ancestors(scene, node):
    """Names of the parent nodes of a node, the direct parent first."""
    out = []
    cur = node
    for _ in range(256):
        if cur.parent is None or not (0 <= cur.parent < len(scene.objs)):
            break
        cur = scene.objs[cur.parent]
        if cur.name is None:
            break
        out.append(cur.name)
    return out


def read_skin(scene, mod, app):
    """Skin modifier data: bones and their bind matrices (modifier chunks), and the
    weights and the bind matrices of the mesh node (local data, chunk 0x2512)."""
    ch = mod.chunk
    nb = struct.unpack('<i', ch.find(0x0020).data[:4])[0]
    tms = [read_matrix3(c) for c in ch.find_all(0x0025)]
    inits = [read_matrix3(c) for c in ch.find_all(0x0480)]
    stretch = [read_matrix3(c) for c in ch.find_all(0x0520)]
    ver = ch.find(0x0230)
    d = ch.find(0x0030).data
    pos = 0
    bones = []
    for i in range(nb):
        nc = struct.unpack_from('<i', d, pos)[0]
        pos += 4 + nc * 12                                # cross sections: u, inner ref, outer ref
        flags, falloff, ref_id, end1, end2 = struct.unpack_from('<BBiii', d, pos)
        pos += 14
        node = scene.objs[mod.refs[ref_id]] if 0 <= ref_id < len(mod.refs) and mod.refs[ref_id] >= 0 else None
        b = {'name': node.name if node is not None else None,
             'node_index': node.index if node is not None else -1,
             'ancestors': _ancestors(scene, node) if node is not None else [],
             'flags': flags}
        if i < len(inits):
            b['init_node_tm'] = inits[i][0]
        if i < len(tms):
            b['inv_init_object_tm'] = tms[i][0]
        if i < len(stretch):
            b['init_stretch_tm'] = stretch[i][0]
        bones.append(b)
    if pos != len(d):
        raise ValueError('Skin %d: bone data has %d bytes, read %d' % (mod.index, len(d), pos))
    out = {'modifier_index': mod.index, 'version': struct.unpack('<i', ver.data[:4])[0] if ver is not None else None,
           'bones': bones}
    ld = app.find(0x2512) if app is not None else None
    if ld is None:
        out['weights'] = None
        return out
    c = ld.find(0x0010)
    if c is not None:
        out['mesh_init_object_tm'] = read_matrix3(c)[0]
    c = ld.find(0x0500)
    if c is not None:
        out['mesh_init_node_tm'] = read_matrix3(c)[0]
    nv = struct.unpack('<i', ld.find(0x0040).data[:4])[0]
    d = ld.find(0x0490).data
    pos = 0
    weights = []
    vflags = []
    for i in range(nv):
        ic, fl = struct.unpack_from('<iI', d, pos)
        pos += 8
        ws = []
        for _ in range(ic):
            bi, w = struct.unpack_from('<if', d, pos)
            pos += 44                 # bone, weight, curve id, segment id, curve u, tangent, point
            ws.append((bi, w))
        weights.append(ws)
        vflags.append(fl)
    if pos != len(d):
        raise ValueError('Skin %d: weight data has %d bytes, read %d' % (mod.index, len(d), pos))
    out['weights'] = weights
    out['vertex_flags'] = vflags
    return out


def read_dismember(scene, mod, app, nfaces):
    """BSDismemberSkin Modifier (NifTools): the faces of each partition and its body part."""
    ld = app.find(0x2512) if app is not None else None
    if ld is None:
        return None
    sets = ld.find(0x2846)
    info = ld.find(0x2849)
    parts = []
    if sets is not None:
        for c in sets.find_all(0x2870):
            b = c.find(0x2700)
            bits = read_bitarray(b.data) if b is not None else np.zeros(0, bool)
            parts.append({'faces': np.nonzero(bits)[0]})
    if info is not None:
        n = struct.unpack_from('<I', info.data, 0)[0]
        for i in range(min(n, len(parts))):
            fl, bp, x = struct.unpack_from('<IiI', info.data, 4 + i * 12)
            parts[i]['part_flags'] = fl
            parts[i]['body_part'] = bp
    return {'modifier_index': mod.index, 'partitions': parts}


# ---------------------------------------------------------------- materials

def _mtl_name(o):
    c = o.chunk.find(0x4000)
    n = c.find(0x4001) if c is not None else None
    return _utf16(n.data) if n is not None else None


def _bitmap_file(scene, o, assets):
    """File path of a Bitmap texture: the asset id in its parameter block, then FileAssetMetaData3."""
    for r in o.refs or []:
        if r < 0:
            continue
        pb = scene.objs[r]
        if not pb.cname.startswith('ParamBlock2') or pb.chunk.children is None:
            continue
        for c in pb.chunk.find_all(0x0003):
            name = c.find(0x1230)
            if name is not None and len(name.data) > 2:
                return _utf16(name.data), None
            link = c.find(0x1260)
            idc = link.find(0x0002) if link is not None and link.children is not None else None
            if idc is not None:
                guid = idc.data[:16].hex()
                if guid in assets:
                    return assets[guid][1], guid
                return None, guid
    return None, None


def read_texmap(scene, idx, assets, depth=0):
    o = scene.objs[idx]
    out = {'class': o.cname, 'name': _mtl_name(o)}
    if classid(o) == CLASS_BITMAP_TEX:
        path, guid = _bitmap_file(scene, o, assets)
        out['file'] = path if path is not None else out['name']
        out['file_source'] = 'asset list' if path is not None else 'map name'
        if guid:
            out['asset_id'] = guid
    elif depth < 4:
        subs = []
        for i, r in enumerate(o.refs or []):
            if r >= 0 and superid(scene.objs[r]) == SUPER_TEXMAP:
                sub = read_texmap(scene, r, assets, depth + 1)
                sub['ref'] = i
                subs.append(sub)
        if subs:
            out['maps'] = subs
            files = [s.get('file') for s in subs if s.get('file')]
            if files:
                out['file'] = files[0]
    return out


def read_material(scene, idx, assets, depth=0):
    """A Standard material or a Multi/Sub-Object material."""
    if idx is None or idx < 0:
        return None
    o = scene.objs[idx]
    out = {'index': idx, 'class': o.cname, 'name': _mtl_name(o)}
    if classid(o) == CLASS_MULTI_MTL or (superid(o) == SUPER_MATERIAL and classid(o) != CLASS_STANDARD_MTL):
        subs = []
        if depth < 3:
            for i, r in enumerate(o.refs or []):
                if r >= 0 and superid(scene.objs[r]) == SUPER_MATERIAL:
                    sub = read_material(scene, r, assets, depth + 1)
                    sub['ref'] = i
                    subs.append(sub)
        out['sub_materials'] = subs
        return out
    shader = None
    texmaps = None
    for r in o.refs or []:
        if r < 0:
            continue
        t = scene.objs[r]
        if t.cname == 'Texmaps':
            texmaps = t
        elif superid(t) == 0x10B0:
            shader = t
    out['shader'] = shader.cname if shader is not None else None
    if shader is not None:
        for r in shader.refs or []:
            if r >= 0 and scene.objs[r].chunk.children is not None:
                for c in scene.objs[r].chunk.find_all(0x100E):
                    # Parameter record: id, type, 11 bytes, value. Type 8 is a string:
                    # byte count, then UTF-16. The Niftools Shader has one: the NIF shader type.
                    if len(c.data) < 19 or struct.unpack_from('<H', c.data, 2)[0] != 8:
                        continue
                    n = struct.unpack_from('<I', c.data, 15)[0]
                    try:
                        text = c.data[19:19 + n].decode('utf-16-le').rstrip('\x00')
                    except UnicodeDecodeError:
                        continue
                    if text:
                        out['nif_shader_type'] = text
    maps = []
    if texmaps is not None:
        refs = texmaps.refs or []
        table = NIF_SLOTS if (shader is not None and shader.cname == 'Niftools Shader') else dict(enumerate(STD_SLOTS))
        for slot in range(len(refs) // 2):
            r = refs[slot * 2 + 1]
            if r < 0:
                continue
            tm = read_texmap(scene, r, assets)
            tm['slot'] = slot
            tm['slot_name'] = table.get(slot)
            maps.append(tm)
    out['maps'] = maps
    return out


# ---------------------------------------------------------------- the scene

def extract(folder, names=None, rest_node='MP7'):
    """Read each node whose object is an Editable Mesh (direct or below modifiers).

    rest_node: name of the node for the rest transform of each mesh node.
    """
    scene = Scene(folder)
    assets = read_assets(folder)
    nodes = []
    for n in scene.nodes():
        if not n.refs or len(n.refs) < 2 or n.refs[1] < 0:
            continue
        if names and n.name not in names:
            continue
        base, mods = resolve_object(scene, n.refs[1])
        if classid(base) != CLASS_EDITABLE_MESH:
            continue
        mesh = read_mesh(base)
        par = scene.objs[n.parent] if n.parent is not None and 0 <= n.parent < len(scene.objs) else None
        pos, rot, scale, axis = node_offset(n)
        item = {
            'name': n.name,
            'index': n.index,
            'parent': par.name if par is not None and par.name else None,
            'parent_index': par.index if par is not None and par.name else -1,
            'object_index': base.index,
            'object_class': base.cname,
            'modifiers': [m.cname for m, _ in mods],
            'object_offset': {'pos': pos, 'rot': rot, 'scale': scale, 'scale_axis': axis,
                              'matrix': offset_matrix(pos, rot, scale, axis)},
            'node_static': node_static(scene, n),
            'parent_static': node_static(scene, par) if par is not None and par.name else None,
            'mesh': mesh,
            'material': read_material(scene, n.refs[3] if len(n.refs) > 3 else -1, assets),
        }
        mesh['normals'] = smooth_normals(mesh['verts'], mesh['faces'], mesh['smoothing_groups'])
        for m, app in mods:
            if classid(m) == CLASS_SKIN:
                item['skin'] = read_skin(scene, m, app)
            elif classid(m) == CLASS_DISMEMBER:
                item['dismember'] = read_dismember(scene, m, app, len(mesh['faces']))
        rest, steps = rest_between(scene, n, rest_node)
        item['rest'] = {'relative_to': rest_node, 'matrix': rest, 'steps': steps}
        nodes.append(item)
    return {'scene': folder, 'nodes': nodes, 'assets': assets}


# ---------------------------------------------------------------- JSON output

NOTES = {
    'matrix': 'Shape (4, 3): rows 0 to 2 are the axes, row 3 is the translation. '
              'A point is a row vector: p2 = p @ m[:3] + m[3].',
    'quaternion': '(x, y, z, w) as the file stores it (class Quat of the 3ds Max SDK). '
                  'tools/maxmath.py quat_to_mat gives its matrix.',
    'units': 'The values are the numbers of the file. The system unit setting of the file is not decoded. '
             'The models come from Fallout: New Vegas NIF files, where 1 unit is 1.4288 cm.',
    'object_offset': 'Chunks 0x096A (pos), 0x096B (rot) and 0x096C (scale, scale_axis) of the node. '
                     'object_offset.matrix maps object space to node space: objectTM = matrix * nodeTM.',
    'mesh.verts': 'Object space.',
    'mesh.faces': 'Three vertex indices for each triangle. The order is counter-clockwise seen from outside.',
    'mesh.face_flags': 'Low 16 bits of the face flags: bits 0 to 2 are the visible edges, bit 3 is FACE_HIDDEN.',
    'mesh.uv': 'Map channel 1, (u, v). The origin is at the bottom left (v goes up). '
               'A NIF or DDS user needs v2 = 1 - v. mesh.uv_faces has one UV index for each face corner.',
    'mesh.normals': 'Computed from the smoothing groups, one unit vector for each face corner '
                    '(faces x 3 x 3), angle weights. Object space.',
    'mesh.spec_normals': 'The normals that the file stores (MeshNormalSpec, from the NIF import). '
                         'mesh.spec_normal_faces has one normal index for each face corner. Object space.',
    'skin.bones[].init_node_tm': 'Chunk 0x0480 (InitNodeTM). Node transform of the bone at the bind: '
                                 'bone space to world space.',
    'skin.bones[].inv_init_object_tm': 'Chunk 0x0025 (BoneData.tm). Inverse of the object transform of the '
                                       'bone at the bind: world space to bone object space.',
    'skin.bones[].init_stretch_tm': 'Chunk 0x0520 (InitStretchTM). Stretch transform of the bone at the bind.',
    'skin.mesh_init_object_tm': 'Chunk 0x0010 of the local data (BaseTM). Object transform of the mesh node '
                                'at the bind: mesh object space to world space.',
    'skin.mesh_init_node_tm': 'Chunk 0x0500 of the local data (BaseNodeTM). Node transform of the mesh node '
                              'at the bind: mesh node space to world space.',
    'skin.weights': 'For each vertex a list of [bone index, weight], as the file stores them. The bone index '
                    'is an index of skin.bones. skin.weight_sum_not_1 lists the vertices whose weights do not '
                    'add up to 1; divide by the sum before use.',
    'skin.bones[].ancestors': 'Names of the parent nodes of the bone in the scene, the direct parent first.',
    'skin.formula': 'Bind position in world space: p_world = p_obj * mesh_init_object_tm. '
                    'Skin offset of a bone (mesh object space to bone space): '
                    'mesh_init_object_tm * inverse(init_node_tm). '
                    'Deformed point in world space: sum of w * (p_world * inv_init_object_tm * objectTM_bone(t)).',
    'dismember': 'BSDismemberSkin Modifier: for each partition the body part number of Fallout 3 and '
                 'New Vegas and the face indices.',
    'rest': 'Node transform relative to the node rest.relative_to (default MP7) from the static values of '
            'the controllers (the values at the save). matrix is null when a controller in the chain is '
            'not static. rest.steps lists the chain.',
    'node_static': 'Static value of the transform controller of the mesh node. matrix is relative to the '
                   'parent node, or to the link target for a Link Constraint.',
    'parent_static': 'The same data for the parent node of the mesh node (null for a child of the scene root).',
}


def _f32list(a):
    """Shortest decimal text that gives the same float32 again."""
    a = np.asarray(a)
    if a.dtype == np.float32:
        flat = [float(str(x)) for x in a.ravel()]
        return np.array(flat, dtype=object).reshape(a.shape).tolist()
    return a.tolist()


def _r(a, digits=7):
    """Rounded list of a computed float64 array."""
    return np.round(np.asarray(a, np.float64), digits).tolist()


def _mat(m):
    return None if m is None else [[float(x) for x in row] for row in np.asarray(m)]


def to_json(data):
    out = {'format': 'maxmesh 1', 'scene': data['scene'], 'notes': NOTES,
           'assets': [{'id': k, 'type': v[0], 'file': v[1]} for k, v in data['assets'].items()],
           'nodes': []}
    for it in data['nodes']:
        mesh = it['mesh']
        uv = mesh['maps'].get(1, {})
        jm = {
            'num_verts': int(len(mesh['verts'])),
            'num_faces': int(len(mesh['faces'])),
            'bbox_min': _f32list(mesh['verts'].min(axis=0)) if len(mesh['verts']) else None,
            'bbox_max': _f32list(mesh['verts'].max(axis=0)) if len(mesh['verts']) else None,
            'verts': _f32list(mesh['verts']),
            'faces': mesh['faces'].tolist(),
            'smoothing_groups': [int(x) for x in mesh['smoothing_groups']],
            'material_ids': mesh['material_ids'].tolist(),
            'face_flags': mesh['face_flags'].tolist(),
            'map_channels': sorted(int(k) for k in mesh['maps'] if k is not None),
            'uv': _f32list(uv['verts'][:, :2]) if 'verts' in uv else None,
            'uv_faces': uv['faces'].tolist() if 'faces' in uv else None,
            'normals': _r(mesh['normals'], 6),
        }
        ns = mesh.get('normal_spec')
        if ns and 'normals' in ns and 'faces' in ns:
            jm['spec_normals'] = _f32list(ns['normals'])
            jm['spec_normal_faces'] = ns['faces'].tolist()
            jm['spec_normal_specified'] = ns['specified'].tolist()
            jm['spec_normal_flags'] = int(ns.get('flags', 0))
            if 'explicit' in ns:
                jm['spec_normals_all_explicit'] = bool(ns['explicit'].all())
        off = it['object_offset']
        jn = {
            'name': it['name'], 'index': it['index'],
            'parent': it['parent'], 'parent_index': it['parent_index'],
            'object_index': it['object_index'], 'object_class': it['object_class'],
            'modifiers': it['modifiers'],
            'object_offset': {'pos': off['pos'].tolist(), 'rot': off['rot'].tolist(),
                              'scale': off['scale'].tolist(), 'scale_axis': off['scale_axis'].tolist(),
                              'matrix': _mat(off['matrix'])},
            'node_static': it['node_static'],
            'parent_static': it['parent_static'],
            'rest': {'relative_to': it['rest']['relative_to'], 'matrix': _mat(it['rest']['matrix']),
                     'steps': it['rest']['steps']},
            'material': it['material'],
            'mesh': jm,
        }
        sk = it.get('skin')
        if sk:
            js = {'modifier_index': sk['modifier_index'], 'version': sk['version'],
                  'bones': [], 'mesh_init_object_tm': _mat(sk.get('mesh_init_object_tm')),
                  'mesh_init_node_tm': _mat(sk.get('mesh_init_node_tm'))}
            for b in sk['bones']:
                js['bones'].append({'name': b['name'], 'node_index': b['node_index'], 'flags': b['flags'],
                                    'parent': b['ancestors'][0] if b['ancestors'] else None,
                                    'ancestors': b['ancestors'],
                                    'init_node_tm': _mat(b.get('init_node_tm')),
                                    'inv_init_object_tm': _mat(b.get('inv_init_object_tm')),
                                    'init_stretch_tm': _mat(b.get('init_stretch_tm'))})
            if sk.get('weights') is not None:
                js['weights'] = [[[int(b), float(str(np.float32(w)))] for b, w in ws] for ws in sk['weights']]
                js['vertex_flags'] = [int(x) for x in sk['vertex_flags']]
                sums = [sum(w for _, w in ws) for ws in sk['weights']]
                js['weight_sum_min'] = round(min(sums), 6) if sums else None
                js['weight_sum_max'] = round(max(sums), 6) if sums else None
                js['weight_sum_not_1'] = [i for i, x in enumerate(sums) if abs(x - 1.0) > 1e-3]
                js['max_influences'] = max((len(ws) for ws in sk['weights']), default=0)
            jn['skin'] = js
        dm = it.get('dismember')
        if dm:
            jn['dismember'] = {'modifier_index': dm['modifier_index'],
                               'partitions': [{'body_part': p.get('body_part'), 'part_flags': p.get('part_flags'),
                                               'faces': p['faces'].tolist()} for p in dm['partitions']]}
        out['nodes'].append(jn)
    return out


def main():
    ap = argparse.ArgumentParser(description='Extract the meshes of a 3ds Max scene (extracted OLE streams).')
    ap.add_argument('folder', help='folder with the streams Scene, ClassDirectory3, DllDirectory')
    ap.add_argument('-o', '--out', required=True, help='output folder, gets meshes.json')
    ap.add_argument('--node', action='append', help='read only this node (repeat for more)')
    ap.add_argument('--rest-node', default='MP7', help='node for the rest transform of each mesh (default MP7)')
    args = ap.parse_args()
    data = extract(args.folder, set(args.node) if args.node else None, args.rest_node)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'meshes.json')
    with open(path, 'w') as fh:
        json.dump(to_json(data), fh, separators=(',', ':'))
    print('%s: %d mesh nodes' % (path, len(data['nodes'])))
    for it in data['nodes']:
        m = it['mesh']
        sk = it.get('skin')
        print('  %-24s parent=%-22s verts=%5d faces=%5d uv=%5d%s' % (
            it['name'], it['parent'], len(m['verts']), len(m['faces']),
            len(m['maps'].get(1, {}).get('verts', [])),
            '  skin bones=%d' % len(sk['bones']) if sk else ''))


if __name__ == '__main__':
    main()
