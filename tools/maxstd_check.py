"""Checks of maxstd.py against the data of the scene files. Usage: python3 maxstd_check.py [ole folder]"""
import math, os, struct, sys, time, zlib
from collections import Counter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import maxmath as mm
from maxscene import Scene
from maxstd import (StdEval, anim_range, time_config, quat_sdk_mul, TAN_NAMES, ORT_NAMES, EULER_NAMES,
                    PB2_TYPE_NAMES)

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get('PORT_WORK', '/tmp'), 'ole')


def qdist(a, b):
    return float(min(np.abs(a - b).max(), np.abs(a + b).max()))


def make_node_tm(s, holder):
    """Stand-in for the node graph: a node with a covered transform controller is evaluated with its
    parent chain. Each other node (CAT) gets a rigid matrix that depends on the node and on the time."""
    depth = [0]

    def stub(i, t):
        rng = np.random.default_rng(zlib.crc32(b'%d' % i))
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = rng.uniform(0, 6.28) + 0.3 * math.sin(t / 2000.0 + i)
        m = mm.quat_to_mat(np.append(ax * math.sin(ang / 2), math.cos(ang / 2)))
        m[3] = rng.uniform(-50, 50, size=3) + 2.0 * math.sin(t / 3000.0)
        return m

    def node_tm(i, t):
        ev = holder[0]
        n = s.objs[i]
        if n.cname != 'Node':
            return mm.ident()
        c = n.refs[0] if n.refs else -1
        if c < 0 or not ev.handles(c) or depth[0] > 80:
            return stub(i, t)
        par = mm.ident()
        depth[0] += 1
        try:
            if ev.kind(c) != 'link' and n.parent is not None and 0 <= n.parent < len(s.objs) and s.objs[n.parent].cname == 'Node':
                par = node_tm(n.parent, t)
            try:
                return ev.apply(c, t, par)
            except NotImplementedError:
                return stub(i, t)
        finally:
            depth[0] -= 1
    return node_tm


def load(folder):
    s = Scene(folder)
    s.folder = folder
    holder = [None]
    ev = StdEval(s, make_node_tm(s, holder))
    holder[0] = ev
    return s, ev


def smoke(name, st):
    """Evaluate each covered controller at 5 times in its key range (or in the animation range)."""
    s, ev = load(os.path.join(ROOT, name))
    a0, a1, tpf = anim_range(s)
    for o in s.objs:
        k = ev.kind(o.index)
        if k is None:
            if o.cls and (o.cls['super'] & 0xff00) == 0x9000 and o.cls['dll'] == -1 or o.cname in ('Noise Rotation', 'LookAt Constraint'):
                st['other'][o.cname] += 1
            continue
        st['class'][o.cname] += 1
        try:
            if k in ('pblock2', 'pblock'):
                for p in ev.pblock2_params(o.index):
                    st['pbtype'][p.type_name] += 1
                ev.pblock2(o.index)
                continue
            if k in ('bezfloat', 'bezscale'):
                d = ev._keydata(o.index)
                for i, fl in enumerate(d['flags']):
                    st['tangent in/out'][(TAN_NAMES[(fl >> 7) & 7], TAN_NAMES[(fl >> 10) & 7])] += 1
                st['ort before/after'][tuple(ORT_NAMES[x] for x in d['ort'])] += 1
                st['track flags'][hex(d['tflags'])] += 1
            if k == 'euler':
                st['euler order chunk'][struct.unpack('<i', o.chunk.find(0x1003).data)[0]] += 1
            if k == 'prs':
                st['inherit flags'][hex(ev.inherit_flags(o.index))] += 1
            if k in ('poscon', 'oricon'):
                p = ev.pblock2(ev._ref(o.index, 0))
                st[o.cname + ' (targets, relative, local)'][(len(p.get(1, [])), p.get(2), p.get(3))] += 1
            if k == 'link':
                st['link targets'][len(ev._link_params(o.index)[0])] += 1
            if k.startswith('noise'):
                d = ev._noise_params(o.index)
                st['noise (frequency, fractal, seed, ramp in, ramp out, range off, >0 flags)'][(round(d['frequency'], 4), d['fractal'], d['seed'], d['rampin'], d['rampout'], d['range_off'], tuple(d['lim']))] += 1
            kt = ev.key_times(o.index)
            lo, hi = (kt[0], kt[-1]) if len(kt) > 1 else (a0, a1)
            for t in np.linspace(lo, hi, 5):
                t = float(t)
                if k in ('bezfloat', 'floatlist', 'linktime', 'noisefloat'):
                    v = np.array([ev.float_value(o.index, t)])
                elif k in ('bezpoint3', 'bezpos', 'posxyz', 'point3list', 'poslist', 'poscon', 'noisepos', 'noisepoint3'):
                    v = ev.point3_value(o.index, t)
                    if k not in ('bezpoint3', 'point3list', 'noisepoint3'):
                        v = np.append(v, ev.apply(o.index, t, mm.ident()).ravel())
                elif k in ('linrot', 'euler', 'rotlist', 'oricon', 'noiserot'):
                    v = np.append(ev.quat_value(o.index, t), ev.apply(o.index, t, mm.ident()).ravel())
                elif k in ('bezscale', 'scalexyz', 'scalelist', 'noisescale'):
                    sv, q = ev.scale_value(o.index, t, True)
                    v = np.concatenate([sv, q, ev.apply(o.index, t, mm.ident()).ravel()])
                else:
                    v = ev.apply(o.index, t, mm.ident()).ravel()
                if not np.all(np.isfinite(v)):
                    st['NOT FINITE'][(name, o.index, o.cname)] += 1
                st['n'][0] += 1
        except NotImplementedError as e:
            msg = str(e)
            st['NotImplementedError'][(o.cname, msg.split('(object')[0].strip() + ' ... ' + msg.split(')', 1)[-1].strip()[:60])] += 1
    for u in ev.unverified:
        st['unverified paths that ran'][u] += 1
    # cached values of the keyframe controllers: value at the time of the time slider
    for o in s.objs:
        if ev.kind(o.index) != 'bezfloat':
            continue
        d = ev._keydata(o.index)
        iv, cur = ev.cached_value(o.index)
        if not d['times'] or iv is None or iv[0] != iv[1]:
            continue
        t = iv[0]
        where = 'at a key' if t in d['times'] else ('between keys' if d['times'][0] < t < d['times'][-1] else 'out of range')
        e = abs(ev.float_value(o.index, t) - cur[0]) / max(1.0, abs(cur[0]))
        st['cache n'][where] += 1
        st['cache max'][where] = max(st['cache max'][where], e)
        i = max(0, min(len(d['times']) - 2, np.searchsorted(d['times'], t, side='right') - 1))
        if where == 'between keys' and (abs(d['outlen'][i] - 0.3333) > 1e-5 or abs(d['inlen'][i + 1] - 0.3333) > 1e-5):
            rel = abs(ev.float_value(o.index, t) - cur[0]) / max(abs(cur[0]), 1e-30)
            st['cache n']['between keys, changed tangent length'] += 1
            st['cache max']['between keys, changed tangent length (relative)'] = max(st['cache max']['between keys, changed tangent length (relative)'], rel)
    return s, ev


def euler_chain(name):
    """Orientation Constraint in a Rotation List [Euler XYZ, constraint]: the stored world offset of a
    bone is Quat(E * M(parent world offset)). Tests the Euler order, E * tm and the Quat handedness."""
    s, ev = load(os.path.join(ROOT, name))
    info = {}
    for n in s.nodes():
        c = n.refs[0]
        if ev.kind(c) != 'prs' or ev.kind(ev._ref(c, 1)) != 'rotlist':
            continue
        conts, _, _ = ev._list(ev._ref(c, 1), 0)
        if [ev.kind(x) for x in conts] != ['euler', 'oricon']:
            continue
        oc = s.objs[conts[1]]
        info[n.index] = (n.parent, conts[0], np.array(struct.unpack('<4f', oc.chunk.find(0x1002).data), float), n.name)
    f = (mm.rotx, mm.roty, mm.rotz)
    res = Counter()
    worst = 0.0
    for idx, (par, e, qw, nm) in info.items():
        if par not in info:
            continue
        ang = [ev.float_value(ev._ref(e, i), 0) for i in range(3)]
        P = mm.quat_to_mat(info[par][2])
        PT = mm.quat_to_mat(mm.quat_inv(info[par][2]))
        tests = {
            'module: Quat(apply(euler, M(parent)))': mm.mat_to_quat(ev.apply(e, 0, P)),
            'other: M(parent) * E': mm.mat_to_quat(mm.mul(P, ev._euler(e, 0))),
            'other: order ZYX': mm.mat_to_quat(mm.mul(mm.mul(mm.mul(f[2](ang[2]), f[1](ang[1])), f[0](ang[0])), P)),
            'other: negative angles': mm.mat_to_quat(mm.mul(mm.mul(mm.mul(f[0](-ang[0]), f[1](-ang[1])), f[2](-ang[2])), P)),
            'other: Quat of the other hand': mm.quat_inv(mm.mat_to_quat(mm.mul(ev._euler(e, 0), PT))),
        }
        res['pairs'] += 1
        for k, q in tests.items():
            d = qdist(q, qw)
            if d < 1e-3:
                res[k] += 1
            if k.startswith('module'):
                worst = max(worst, d)
    return res, worst


def fo4rig_checks():
    """Constraints that replaced a controller (Copy): world offset = local offset * parent rotation,
    world point = local point * parent matrix. Tests Quat::operator* and Point3 * MakeMatrix."""
    folder = os.path.join(ROOT, 'fo4rig')
    if not os.path.isdir(folder):
        return None
    s, ev = load(folder)
    info = {}
    for n in s.nodes():
        c = n.refs[0]
        if ev.kind(c) != 'prs' or ev.kind(ev._ref(c, 1)) != 'oricon':
            continue
        oc = s.objs[ev._ref(c, 1)]
        q = lambda cid: np.array(struct.unpack('<4f', oc.chunk.find(cid).data), float)
        d = {'parent': n.parent, 'ql': q(0x1001), 'qw': q(0x1002)}
        if ev.kind(ev._ref(c, 0)) == 'poscon':
            pc = s.objs[ev._ref(c, 0)]
            p = lambda cid: np.array(struct.unpack('<3f', pc.chunk.find(cid).data), float)
            d['pl'] = p(0x1001)
            d['pw'] = p(0x1002)
        info[n.index] = d
    r = Counter()
    wq = wp = 0.0
    for idx, d in info.items():
        par = info.get(d['parent'])
        if par is None:
            continue
        ang = 2 * math.degrees(math.acos(min(1.0, abs(d['ql'][3]))))
        if ang > 2.0:
            r['quat pairs (local offset > 2 degrees)'] += 1
            a = qdist(mm.quat_norm(quat_sdk_mul(d['ql'], par['qw'])), d['qw'])
            b = qdist(mm.quat_norm(quat_sdk_mul(par['qw'], d['ql'])), d['qw'])
            r['module: world = quat_sdk_mul(local, parent)'] += a < 1e-4
            r['other: world = quat_sdk_mul(parent, local)'] += b < 1e-4
            wq = max(wq, a)
        if 'pl' in d and 'pw' in par and np.linalg.norm(d['pl']) > 1.0 and ang > 2.0:
            M = mm.quat_to_mat(par['qw'])
            M[3] = par['pw']
            r['point pairs'] += 1
            a = float(np.abs(mm.ptrans(d['pl'], M) - d['pw']).max())
            b = float(np.abs(d['pl'] @ M[:3].T + par['pw'] - d['pw']).max())
            r['module: world point = ptrans(local, quat_to_mat(parent))'] += a < 1e-3
            r['other: transposed matrix'] += b < 1e-3
            wp = max(wp, a)
    return r, wq, wp


if __name__ == '__main__':
    names = [l.split('|')[0] for l in open(os.path.join(ROOT, 'index.txt')).read().split('\n') if '|' in l]
    st = {}

    class Stats(dict):
        def __missing__(self, k):
            self[k] = [0] if k == 'n' else (Counter() if k != 'cache max' else Counter())
            return self[k]
    st = Stats()
    t0 = time.time()
    print('scene                          fps  start    end  frames  slider')
    for nm in names:
        tc = time_config(os.path.join(ROOT, nm))
        print('%-30s %3d %6d %6d %7.1f %7d' % (nm, tc['fps'], tc['start'], tc['end'], (tc['end'] - tc['start']) / tc['ticks_per_frame'], tc['slider']))
        smoke(nm, st)
    print('\n%d scenes, %d evaluations, %.0f s' % (len(names), st['n'][0], time.time() - t0))
    for key in sorted(k for k in st if k not in ('n',)):
        print('\n[%s]' % key)
        for k, v in sorted(st[key].items(), key=lambda kv: str(kv[0])):
            print('  %10s  %s' % (('%.3g' % v) if isinstance(v, float) else v, k))
    print('\n[euler chain check]')
    tot = Counter()
    worst = 0.0
    for nm in names:
        r, w = euler_chain(nm)
        tot.update(r)
        worst = max(worst, w)
    for k, v in tot.items():
        print('  %6d  %s' % (v, k))
    print('  worst error of the module rule: %.2g' % worst)
    r = fo4rig_checks()
    if r:
        print('\n[fo4rig: constraints with a local offset]')
        for k, v in r[0].items():
            print('  %6d  %s' % (v, k))
        print('  worst quaternion error %.2g, worst point error %.2g' % (r[1], r[2]))
