"""Evaluate a 3ds Max scene and write the matrix of each node for each frame.

    python3 tools/maxbake.py ole/SCENE -o out/anim/SCENE.json [--fps 30] [--start F] [--end F]

The output has, for each node, its parent and its world matrix for each sample time.
A matrix is 12 numbers: the three axis rows and the translation row (3ds Max layout).
"""
import argparse
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maxscene import Scene
import maxmath as mm
import maxcat


def load(folder):
    from maxstd import StdEval
    sc = Scene(folder)
    sc.folder = folder
    holder = {}
    std = StdEval(sc, lambda idx, t: holder['rig'].node_tm(idx, t))
    rig = maxcat.Rig(sc, std)
    holder['rig'] = rig
    std.noise_off = os.environ.get('MAX_NOISE_OFF') == '1'
    return sc, std, rig


def wanted(name):
    if name is None:
        return False
    return True


def bake(folder, fps=30, start=None, end=None, names=None, verbose=False):
    sc, std, rig = load(folder)
    if start is None or end is None:
        import maxstd
        s0, e0, tpf = maxstd.anim_range(folder)
        start = s0 if start is None else start
        end = e0 if end is None else end
    step = mm.TICKS_PER_SEC // fps
    times = list(range(int(start), int(end) + 1, step))
    if times[-1] != int(end):
        times.append(int(end))
    nodes = [o for o in sc.nodes() if wanted(o.name)]
    if names:
        nodes = [o for o in nodes if o.name in names]
    out = {}
    for o in nodes:
        p = rig.node_parent(o.index)
        out[o.name if o.name not in out else '%s#%d' % (o.name, o.index)] = {
            'index': o.index, 'parent': sc.objs[p].name if p is not None else None, 'world': []}
    keys = list(out.keys())
    errors = {}
    for t in times:
        for k in keys:
            idx = out[k]['index']
            try:
                tm = rig.node_tm(idx, t)
                out[k]['world'].append([round(float(v), 6) for v in tm.reshape(-1)])
            except Exception as e:  # keep the other nodes
                errors.setdefault(k, '%s: %s' % (type(e).__name__, e))
                out[k]['world'].append(None)
        if verbose:
            print('t', t, file=sys.stderr)
    return {'scene': os.path.basename(os.path.normpath(folder)), 'fps': fps, 'ticks_per_second': mm.TICKS_PER_SEC,
            'start': int(start), 'end': int(end), 'times': times, 'nodes': out, 'errors': errors}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('folder')
    ap.add_argument('-o', '--output', required=True)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--start', type=int)
    ap.add_argument('--end', type=int)
    a = ap.parse_args()
    data = bake(a.folder, a.fps, a.start, a.end)
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    with open(a.output, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    print('frames', len(data['times']), 'nodes', len(data['nodes']), 'errors', len(data['errors']))
    for k, v in list(data['errors'].items())[:40]:
        print('  ', k, '->', v)
