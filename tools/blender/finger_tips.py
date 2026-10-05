"""Measure the finger bones of the two hand meshes: the tip length of each last finger
bone (the farthest skin vertex along the bone), the radius of each finger segment and the
center of its skin about the bone axis. Writes rig/finger_tips.json. Run with Blender (it uses the mesh readers of the render scripts)."""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(HERE))
from render_fo4 import SkinnedMesh, FO4

def section(loc):
    """The extent of the skin across the bone (local Y and Z), from the vertices of the middle
    part of the bone. The axes are those of the FO4 bone."""
    x0, x1 = loc[:, 0].min(), loc[:, 0].max()
    mid = loc[(loc[:, 0] > x0 + 0.25 * (x1 - x0)) & (loc[:, 0] < x0 + 0.75 * (x1 - x0))]
    if len(mid) < 4:
        mid = loc
    return {'y_min': float(mid[:, 1].min()), 'y_max': float(mid[:, 1].max()),
            'z_min': float(mid[:, 2].min()), 'z_max': float(mid[:, 2].max())}


def centers(loc):
    """The center of the skin about the bone axis (local Y and Z), in four parts along the bone:
    in each part the middle of the extent of the skin. Returns the mean of the two middle parts
    (the center of the finger segment) and the value of the last part (the fingertip of a last
    bone). The skin of a finger is not a tube about its bone: the thumb of the Fallout 4 hand
    has its center 0.4 units to the pad side."""
    edges = np.linspace(0.0, loc[:, 0].max(), 5)
    parts = []
    for a, b in zip(edges[:-1], edges[1:]):
        s = loc[(loc[:, 0] >= a) & (loc[:, 0] <= b)]
        parts.append(None if len(s) < 3 else [(s[:, 1].min() + s[:, 1].max()) / 2, (s[:, 2].min() + s[:, 2].max()) / 2])
    whole = [(loc[:, 1].min() + loc[:, 1].max()) / 2, (loc[:, 2].min() + loc[:, 2].max()) / 2]
    middle = [m for m in parts[1:3] if m is not None] or [whole]
    last = parts[3] or middle[-1]
    return {'center': [round(float(np.mean([m[i] for m in middle])), 4) for i in (0, 1)],
            'tip_center': [round(float(last[i]), 4) for i in (0, 1)]}


out = {'fnv': {}, 'fo4': {}}
sm = SkinnedMesh(os.path.join(FO4, '1stpersonmalehands.glb'))
main = sm.joints[np.arange(len(sm.verts)), np.argmax(sm.weights, axis=1)]
for j, jn in enumerate(sm.joint_names):
    if 'Finger' not in jn:
        continue
    sel = main == j
    if not sel.any():
        continue
    loc = sm.verts[sel] @ sm.inv_bind[j][:3, :3].T + sm.inv_bind[j][:3, 3]
    out['fo4'][jn] = {'x_max': float(loc[:, 0].max()), 'x_min': float(loc[:, 0].min()),
                     'radius': float(np.percentile(np.linalg.norm(loc[:, 1:], axis=1), 80)), 'verts': int(sel.sum())}
    out['fo4'][jn].update(section(loc))
    out['fo4'][jn].update(centers(loc))
md = json.load(open(os.path.join(os.environ.get('PORT_RIG') or os.path.join(os.path.dirname(os.path.dirname(HERE)), 'rig'), 'fnv_arms.json')))
for nd in md['nodes']:
    if nd['name'] not in ('RightHand:0', 'LeftHand:0'):
        continue
    sk = nd['skin']
    verts = np.array(nd['mesh']['verts'], float)
    b = np.array(sk['mesh_init_object_tm'], float)
    pw = verts @ b[:3] + b[3]
    w = np.zeros((len(verts), len(sk['bones'])))
    for i, lst in enumerate(sk['weights']):
        for bi, wt in lst:
            w[i, bi] += wt
    main = np.argmax(w, axis=1)
    for j, bone in enumerate(sk['bones']):
        if 'Finger' not in bone['name'] and 'Thumb' not in bone['name']:
            continue
        sel = main == j
        if not sel.any():
            continue
        inv = np.array(bone['inv_init_object_tm'], float)
        loc = pw[sel] @ inv[:3] + inv[3]
        out['fnv'][bone['name']] = {'x_max': float(loc[:, 0].max()), 'x_min': float(loc[:, 0].min()),
                                   'radius': float(np.percentile(np.linalg.norm(loc[:, 1:], axis=1), 80)), 'verts': int(sel.sum())}
        # the FO4 convention of the bone axes: a half turn about X
        out['fnv'][bone['name']].update(section(loc * np.array([1.0, -1.0, -1.0])))
        out['fnv'][bone['name']].update(centers(loc * np.array([1.0, -1.0, -1.0])))
json.dump(out, open(os.path.join(FO4, 'finger_tips.json'), 'w'), indent=1)
for k in ('fnv', 'fo4'):
    for n in sorted(out[k]):
        if n.startswith(('Bip01 R', 'RArm')):
            v = out[k][n]
            print('%-4s %-22s x %5.2f .. %5.2f  radius %.2f  y %5.2f .. %5.2f  z %5.2f .. %5.2f  verts %d' % (
                k, n, v['x_min'], v['x_max'], v['radius'], v['y_min'], v['y_max'], v['z_min'], v['z_max'], v['verts']))
