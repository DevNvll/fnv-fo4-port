"""Measure how far the hand meshes go into the gun meshes.

blender --background --factory-startup --python hand_contact.py -- --scene reload_48 [--frames 0,30] [--kind fo4|src|both]

For each frame and each hand bone: the count of skin vertices inside a gun part and the
largest depth (game units). A vertex belongs to the bone with its largest weight.
fo4: out/fo4/poses/SCENE.npz (or a clip in out/fo4/hkx) with the vanilla FO4 hand mesh.
src: out/anim/SCENE.json with the FNV hand meshes and the gun meshes of the source
(tools/source_meshes.py gives the file name).
PORT_ATTACHMENTS=1 adds the attachment meshes to the gun parts of the fo4 check.
[validate] of the configuration file: skip_parts (shapes of the parts file that are not
colliders), late_parts (shapes that are added after the others) and skip_source (source meshes).
"""
import argparse
import json
import os
import sys
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import maxmath as mm  # noqa: E402
import port_config as cfg  # noqa: E402
import source_meshes  # noqa: E402

ROOT = cfg.WORK
VAL = cfg.D.get('validate', {})
DIRS = [Vector((0.577, 0.577, 0.577)), Vector((-0.8, 0.27, 0.53)), Vector((0.2, -0.9, 0.38))]


def inside(tree, p):
    """Parity of ray hits in three directions."""
    votes = 0
    for d in DIRS:
        n = 0
        o = Vector(p)
        for _ in range(64):
            loc, nrm, idx, dist = tree.ray_cast(o, d)
            if loc is None:
                break
            n += 1
            o = loc + d * 1e-4
        votes += n & 1
    return votes >= 2


class Parts:
    """Gun parts as BVH trees in a local space, with a function that gives the matrix
    (4x4, column vectors) from world space to that local space for a frame."""

    def __init__(self):
        self.items = []

    def add(self, name, verts, tris, to_local):
        tree = BVHTree.FromPolygons([tuple(v) for v in verts], [tuple(t) for t in tris], epsilon=0.0)
        lo = np.min(verts, axis=0) - 0.05
        hi = np.max(verts, axis=0) + 0.05
        self.items.append((name, tree, lo, hi, to_local))

    def depth(self, pts, frame):
        """pts: N x 3 world points. Returns (depth N, part index N)."""
        depth = np.zeros(len(pts))
        which = np.full(len(pts), -1)
        for k, (name, tree, lo, hi, to_local) in enumerate(self.items):
            m = to_local(frame)
            loc = pts @ m[:3, :3].T + m[:3, 3]
            cand = np.nonzero(np.all((loc >= lo) & (loc <= hi), axis=1))[0]
            for i in cand:
                p = Vector(loc[i])
                if inside(tree, p):
                    near = tree.find_nearest(p)
                    d = near[3] if near[0] is not None else 0.0
                    if d > depth[i]:
                        depth[i] = d
                        which[i] = k
        return depth, which


def report(label, frames, groups, names, fn_points, parts, thr=0.05):
    rows = []
    for f in frames:
        pts = fn_points(f)
        depth, which = parts.depth(pts, f)
        per = {}
        for g, sel in groups.items():
            d = depth[sel]
            n = int((d > thr).sum())
            if n:
                k = which[sel][np.argmax(d)]
                per[g] = (n, round(float(d.max()), 2), parts.items[k][0])
        tot = int((depth > thr).sum())
        rows.append((f, tot, round(float(depth.max()), 2), per))
        print('%s frame %3d: %4d vertices inside, max depth %.2f' % (label, f, tot, depth.max()))
        for g in sorted(per, key=lambda g: -per[g][1])[:12]:
            print('      %-22s %4d verts  depth %.2f  in %s' % (g, per[g][0], per[g][1], per[g][2]))
    return rows


def run_fo4(scene, frames, hand_only=True):
    from render_fo4 import SkinnedMesh, Helpers, GUESS, FO4
    path = os.path.join(ROOT, os.environ.get('PORT_SET', 'out/fo4'), 'poses', scene + '.npz')
    if not os.path.exists(path):
        path = os.path.join(ROOT, 'out/fo4/hkx', scene + '.npz')
    d = np.load(path)
    poses = d['poses']
    names = [str(x) for x in d['names']]
    idx = {n: i for i, n in enumerate(names)}
    pj = json.load(open(os.path.join(ROOT, 'out/fo4mesh/parts.json')))
    parts = Parts()
    for s in pj['shapes']:
        if s['name'] in VAL.get('skip_parts', ()) or s['name'] in VAL.get('late_parts', ()):
            continue                      # for example a second magazine at the same place, or a cartridge that is inside
        bi = idx[s['bone']]
        parts.add(s['name'], np.array(s['positions']), s['triangles'], lambda f, bi=bi: np.linalg.inv(poses[f, bi]))
    # with PORT_ATTACHMENTS=1 also the attachment meshes of tools/attachments.py (all on the gun at one time)
    extra = os.path.join(ROOT, 'out/fo4mesh/attachments.json')
    if os.environ.get('PORT_ATTACHMENTS') == '1' and os.path.exists(extra):
        for at in json.load(open(extra))['attachments']:
            for s in at['shapes']:
                bi = idx[s['bone']]
                parts.add(s['name'], np.array(s['positions']), s['triangles'], lambda f, bi=bi: np.linalg.inv(poses[f, bi]))
    # the second magazine only when it is away from the first
    for s in pj['shapes']:
        if s['name'] in VAL.get('late_parts', ()):
            bi = idx[s['bone']]
            parts.add(s['name'], np.array(s['positions']), s['triangles'], lambda f, bi=bi: np.linalg.inv(poses[f, bi]))
    sm = SkinnedMesh(os.path.join(FO4, '1stpersonmalehands.glb'))
    helpers = Helpers(names)
    rules = []
    for jn in sm.joint_names:
        if jn in idx:
            rules.append((idx[jn], np.eye(4)))
            continue
        ch = helpers.chain(jn)
        if ch is None:
            base = GUESS.get(jn) or jn.replace('_skin', '')
            wb = sm.bind_world.get(base)
            if wb is None:
                wb = helpers.rest_world(base)
            rules.append((idx[base], np.linalg.inv(wb) @ sm.bind_world[jn]))
        else:
            rules.append((idx[ch[0]], ch[1]))
    main = sm.joints[np.arange(len(sm.verts)), np.argmax(sm.weights, axis=1)]
    groups = {}
    for j, jn in enumerate(sm.joint_names):
        sel = np.nonzero(main == j)[0]
        if len(sel) and ('Finger' in jn or 'Hand' in jn):
            groups[jn] = sel

    def points(f):
        pw = poses[f]
        out = np.zeros_like(sm.verts)
        mats = [pw[bi] @ extra @ sm.inv_bind[j] for j, (bi, extra) in enumerate(rules)]
        for k in range(sm.joints.shape[1]):
            jj = sm.joints[:, k]
            ww = sm.weights[:, k]
            for j in np.unique(jj):
                sel = (jj == j) & (ww > 0)
                if sel.any():
                    m = mats[j]
                    out[sel] += ww[sel, None] * (sm.verts[sel] @ m[:3, :3].T + m[:3, 3])
        return out
    n = poses.shape[0]
    frames = [min(f, n - 1) for f in frames]
    return report('FO4 ' + scene, frames, groups, names, points, parts)


def run_src(scene, frames):
    bake = json.load(open(os.path.join(ROOT, 'out/anim', scene + '.json')))
    nodes = bake['nodes']
    md = json.load(open(source_meshes.path()))

    def tm(name, f):
        return np.array(nodes[name]['world'][f], float).reshape(4, 3)

    def col(m43):
        m = np.eye(4)
        m[:3, :3] = m43[:3].T
        m[:3, 3] = m43[3]
        return m
    parts = Parts()
    seen = set()
    for nd in md['nodes']:
        nm = nd['name']
        if nd.get('skin') or nm in VAL.get('skip_source', ()):
            continue
        verts = np.array(nd['mesh']['verts'], float)
        off = np.array(nd['object_offset']['matrix'], float)
        parts.add(nm, verts, nd['mesh']['faces'], lambda f, nm=nm, off=off: np.linalg.inv(col(mm.mul(off, tm(nm, f)))))
    hands = [n for n in md['nodes'] if n['name'] in ('RightHand:0', 'LeftHand:0')]
    prep = []
    groups = {}
    base = 0
    for nd in hands:
        sk = nd['skin']
        verts = np.array(nd['mesh']['verts'], float)
        b = np.array(sk['mesh_init_object_tm'], float)
        pw = verts @ b[:3] + b[3]
        bones = [(x['name'], np.array(x['inv_init_object_tm'], float)) for x in sk['bones']]
        w = np.zeros((len(verts), len(bones)))
        for i, lst in enumerate(sk['weights']):
            tot = sum(x[1] for x in lst) or 1.0
            for bi, wt in lst:
                w[i, bi] += wt / tot
        main = np.argmax(w, axis=1)
        for j, (bn, _) in enumerate(bones):
            sel = np.nonzero(main == j)[0]
            if len(sel):
                groups[bn] = sel + base
        prep.append((pw, bones, w))
        base += len(verts)

    def points(f):
        outs = []
        for pw, bones, w in prep:
            out = np.zeros_like(pw)
            for bi, (bn, invinit) in enumerate(bones):
                c = w[:, bi]
                if c.any():
                    m = mm.mul(invinit, tm(bn, f))
                    out += c[:, None] * (pw @ m[:3] + m[3])
            outs.append(out)
        return np.vstack(outs)
    n = len(bake['times'])
    frames = [min(f, n - 1) for f in frames]
    return report('SRC ' + scene, frames, groups, None, points, parts)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True)
    ap.add_argument('--frames', default='0')
    ap.add_argument('--kind', default='both')
    a = ap.parse_args(sys.argv[sys.argv.index('--') + 1:])
    frames = [int(x) for x in a.frames.split(',')]
    if a.kind in ('src', 'both'):
        run_src(a.scene, frames)
    if a.kind in ('fo4', 'both'):
        run_fo4(a.scene, frames)
