"""Check one scene: the CAT chain against the scene skeleton for each third frame."""
import sys, time, traceback
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import maxbake, maxstd, maxmath as mm, maxcat
folder=sys.argv[1]
sc, std, rig = maxbake.load(folder)
std.noise_off = True
names={}
for o in sc.nodes(): names.setdefault(o.name, o.index)
tc=maxstd.time_config(folder)
pairs=[('Bip01 Pelvis','Base HumanPelvis001'),('Bip01 Spine','Base HumanSpine1'),('Bip01 Neck','Base HumanPelvis002')]
for S,cs in (('R','RArm'),('L','LArm')):
    pairs += [('Bip01 %s Clavicle'%S,'Base Human%sCollarbone'%cs),('Bip01 %s UpperArm'%S,'Base Human%sUpperarmTwist'%cs),('Bip01 %s Forearm'%S,'Base Human%sForearmupper'%cs),('Bip01 %s Hand'%S,'Base Human%sPalm'%cs)]
t0=time.time(); worst=(0,None,None); errs={}
tpf=tc['ticks_per_frame']
frames=list(range(tc['start']//tpf, tc['end']//tpf+1, int(sys.argv[2]) if len(sys.argv)>2 else 3))
limbs=[o for o in rig.objs.values() if isinstance(o, maxcat.LimbData2) and not o.is_leg]
ik=set(); ta=set(); ikp=set()
for f in frames:
    t=f*tpf
    try:
        mx=0; who=None
        for fn,cn in pairs:
            a=rig.node_tm(names[fn], t); b=rig.node_tm(names[cn], t)
            d=float(np.linalg.norm(a[3]-b[3]))
            if d>mx: mx=d; who=fn
        if mx>worst[0]: worst=(round(mx,4),f,who)
        for l in limbs:
            ik.add(round(l.get_ikfk_ratio(t),3)); ta.add(round(l.palm.get_target_align(t),3)); ikp.add(round(l.get_limb_ik_pos(t),3))
    except Exception as e:
        k='%s: %s'%(type(e).__name__, e)
        if k not in errs:
            errs[k]=f
            traceback.print_exc(limit=-6)
print(folder, 'fps', tc['fps'], 'frames %d..%d'%(frames[0], frames[-1]), 'worst', worst, 'ikfk', sorted(ik), 'talign', sorted(ta), 'ikpos', sorted(ikp), 'errors', errs, 'time %.1fs'%(time.time()-t0), 'unverified', sorted(std.unverified))
