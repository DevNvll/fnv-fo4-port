"""Decode a Fallout 4 HKX clip and write the world matrix of each bone for each frame.

    python3 tools/hkx_to_npz.py CLIP.hkx -o OUT.npz [--skeleton fo4/1st_skeleton.hkx]
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import anim_fo4
from port_hkx import world_from_local

ap = argparse.ArgumentParser()
ap.add_argument('clip')
ap.add_argument('-o', '--output', required=True)
ap.add_argument('--skeleton', default=os.path.join(os.environ.get('PORT_RIG') or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rig'), '1st_skeleton.hkx'))
a = ap.parse_args()
sk = anim_fo4.load_fo4_skeleton(a.skeleton)
an = anim_fo4.load_fo4_animation(a.clip)
nb = len(sk.bones)
n = an.num_frames
tr = np.zeros((n, nb, 3)); ro = np.zeros((n, nb, 4)); sc = np.ones((n, nb, 3))
for j in range(nb):
    rp = sk.reference_pose[j]
    tr[:, j] = rp.translation; ro[:, j] = rp.rotation; sc[:, j] = rp.scale
t2b = an.track_to_bone_indices or list(range(an.num_tracks))
for ti, bi in enumerate(t2b):
    t = an.tracks[ti]
    tr[:, bi] = np.array(t.translations); ro[:, bi] = np.array(t.rotations); sc[:, bi] = np.array(t.scales)
poses = world_from_local(tr, ro, sc, sk.parents)
np.savez_compressed(a.output, poses=poses, names=np.array(sk.bones), parents=np.array(sk.parents), fps=round(1.0 / an.frame_duration))
print(os.path.basename(a.clip), 'frames', n, 'duration', round(an.duration, 4), 'annotations', [(round(x.time, 3), x.text) for x in an.annotations])
