"""Compare the rest geometry of the FNV hand and the FO4 hand in the space of the hand bone."""
import json, math, os, sys
import numpy as np
from mathutils import Matrix, Quaternion, Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(HERE))
from common import load_target, load_source_rest
from solver import ARM_ROLL, finger_chains

RIG = os.environ.get('PORT_RIG') or os.path.join(os.path.dirname(os.path.dirname(HERE)), 'rig')
skel, target = load_target(os.path.join(RIG, '1st_skeleton.hkx'))
source = load_source_rest(os.path.join(RIG, 'nvcs_1st.json'))
tips = json.load(open(os.path.join(RIG, 'finger_tips.json')))
roll = ARM_ROLL.to_matrix().to_4x4()

def fo4_world(name):
    m = Matrix.Identity(4)
    chain = []
    n = name
    while n:
        chain.append(n); n = target.parents[n]
    for n in reversed(chain):
        m = m @ target.local[n]
    return m

for side in ('L', 'R'):
    hs = source.rest['Bip01 %s Hand' % side] @ roll
    ht = fo4_world('%sArm_Hand' % side)
    print('side', side)
    A = []; B = []
    for (s, finger), (fo4, fnv) in finger_chains().items():
        if s != side: continue
        ps = [(hs.inverted() @ source.rest[n]).translation for n in fnv]
        pt = [(ht.inverted() @ fo4_world(n)).translation for n in fo4]
        ps.append((hs.inverted() @ source.rest[fnv[2]]) @ Vector((tips['fnv'][fnv[2]]['x_max'], 0, 0)))
        pt.append((ht.inverted() @ fo4_world(fo4[2])) @ Vector((tips['fo4'][fo4[2]]['x_max'], 0, 0)))
        print(' finger %d  fnv base (%6.2f %6.2f %6.2f) len %.2f %.2f %.2f r %.2f| fo4 base (%6.2f %6.2f %6.2f) len %.2f %.2f %.2f r %.2f| base diff (%5.2f %5.2f %5.2f)' % (
            finger, *ps[0], (ps[1]-ps[0]).length, (ps[2]-ps[1]).length, (ps[3]-ps[2]).length, tips['fnv'][fnv[1]].get('radius', 0),
            *pt[0], (pt[1]-pt[0]).length, (pt[2]-pt[1]).length, (pt[3]-pt[2]).length, tips['fo4'][fo4[1]].get('radius', 0), *(pt[0]-ps[0])))
        if finger > 1:
            A.append(np.array(ps[0])); B.append(np.array(pt[0]))
    A = np.array(A); B = np.array(B)
    d = (A - B).mean(axis=0)
    print('  mean knuckle shift (fnv - fo4) in hand space', np.round(d, 3), 'residual', np.round(np.linalg.norm(A - (B + d), axis=1), 3))
    # rigid fit (Kabsch) of FO4 knuckles on FNV knuckles
    ca, cb = A.mean(0), B.mean(0)
    H = (B - cb).T @ (A - ca)
    U, S, Vt = np.linalg.svd(H)
    dd = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1, 1, dd]) @ U.T
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(Rm) - 1) / 2))))
    res = np.linalg.norm(A - ((B - cb) @ Rm.T + ca), axis=1)
    print('  rigid fit angle %.1f deg residual' % ang, np.round(res, 3), 'spread fnv', round(float(np.linalg.norm(A[0]-A[-1])),2), 'fo4', round(float(np.linalg.norm(B[0]-B[-1])),2))
