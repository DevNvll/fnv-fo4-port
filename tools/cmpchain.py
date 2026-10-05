import sys, json
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import maxbake, maxstd, maxmath as mm, maxcat
folder=sys.argv[1] if len(sys.argv)>1 else 'ole/reload_48'
t=int(sys.argv[2]) if len(sys.argv)>2 else 0
sc, std, rig = maxbake.load(folder)
names={o.name:o.index for o in sc.nodes()}
np.set_printoptions(precision=3, suppress=True)
pairs=[('Bip01 Pelvis','Base HumanPelvis001'),('Bip01 Spine','Base HumanSpine1'),('Bip01 Spine1','Base HumanSpine2'),('Bip01 Spine2','Base HumanSpine3'),('Bip01 Neck','Base HumanPelvis002')]
for S,cs in (('R','RArm'),('L','LArm')):
    pairs += [('Bip01 %s Clavicle'%S,'Base Human%sCollarbone'%cs),('Bip01 %s UpperArm'%S,'Base Human%sUpperarmTwist'%cs),('Bip01 %s Forearm'%S,'Base Human%sForearmupper'%cs),('Bip01 %s Hand'%S,'Base Human%sPalm'%cs)]
pairs += [('Weapon','Base HumanCatWeaponBone'),('Camera1st','Base HumanCambone')]
mx=0
for fn,cn in pairs:
    a=rig.node_tm(names[fn], t); b=rig.node_tm(names[cn], t)
    d=np.linalg.norm(a[3]-b[3]); mx=max(mx,d)
    print('%-18s scene=%-28s cat=%-28s dist=%.3f'%(fn, a[3], b[3], d))
print('max dist', round(mx,3))
for n in ('Base HumanRArmIKTarget','Base HumanLArmIKTarget','Base HumanRArmUpVectorNode','Base HumanLArmUpVectorNode','Weapon','MP7','##nmClip','##nmClip2','##nmBar','Bip01 R Finger1','Bip01 L Finger1'):
    print('%-28s %s'%(n, rig.node_tm(names[n], t)[3]))
for i,o in rig.objs.items():
    if isinstance(o, maxcat.LimbData2):
        print('limb #%d flags=%x lmr=%d leg=%s ikfk=%.3f ikpos=%.3f ffb=%.3f bones=%d palm=%s iktarget=%s upnode=%s talign=%s'%(i,o.flags,o.lmr,o.is_leg,o.get_ikfk_ratio(t),o.get_limb_ik_pos(t),o.get_force_feedback(t),o.num_bones,o.palm is not None, o.iktarget, o.upnode, o.palm.get_target_align(t) if o.palm else None))
print('unverified:', sorted(std.unverified))
