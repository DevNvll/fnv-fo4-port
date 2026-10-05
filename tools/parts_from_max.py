"""Convert the gun meshes of a 3ds Max scene to Fallout 4 weapon space.

    python3 tools/parts_from_max.py --meshes out/mesh/SCENE/meshes.json --rest out/anim/SCENE.json \
        -o out/fo4mesh/parts.json

Each shape has its vertices in the local space of the FO4 weapon bone that carries it
(see port_config.py). One vertex for each different (position, UV, normal) corner.
"""
import argparse
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maxmath as mm
import port_config as cfg

FNV_BONE = {bone: node for bone, (node, _) in cfg.PARTS.items()}
MUZZLE_MESH = cfg.D.get('weapon', {}).get('muzzle_mesh')


def convert(mesh_path, rest_path, frame=0):
    md = json.load(open(mesh_path))
    bake = json.load(open(rest_path))
    nodes = bake['nodes']

    def tm(name):
        return np.array(nodes[name]['world'][frame], float).reshape(4, 3)
    shapes = []
    for n in md['nodes']:
        name = n['name']
        if n.get('skin'):
            continue
        bone = cfg.MESH_BONE.get(name, 'Weapon')
        if bone == 'Weapon' and name not in cfg.STATIC_MESHES:
            raise SystemExit('mesh with no rule: ' + name)
        me = n['mesh']
        off = np.array(n['object_offset']['matrix'], float)
        rel = mm.mul(mm.mul(off, tm(name)), mm.inv(tm(FNV_BONE[bone])))     # object space -> FNV bone space (row vectors)
        verts = np.array(me['verts'], float) @ rel[:3] + rel[3]
        rot = mm.no_scale(rel)[:3]
        piv = cfg.pivot_in_weapon(bone)
        pos = cfg.fnv_to_fo4(verts) - piv
        faces = np.array(me['faces'], int)
        uv = np.array(me['uv'], float)
        uvf = np.array(me['uv_faces'], int)
        if me.get('spec_normals_all_explicit') and me.get('spec_normals'):
            sn = np.array(me['spec_normals'], float)
            snf = np.array(me['spec_normal_faces'], int)
            corner_normals = sn[snf]                                           # faces x 3 x 3
        else:
            corner_normals = np.array(me['normals'], float)
        cn = corner_normals.reshape(-1, 3) @ rot                               # object -> FNV bone space
        cn = cn @ cfg.R.T
        ln = np.linalg.norm(cn, axis=1, keepdims=True)
        cn = cn / np.where(ln > 0, ln, 1)
        cn = cn.reshape(-1, 3, 3)
        index = {}
        out_p, out_n, out_uv, tris = [], [], [], []
        for fi in range(len(faces)):
            tri = []
            for c in range(3):
                vi = int(faces[fi, c])
                ti = int(uvf[fi, c])
                nrm = cn[fi, c]
                key = (vi, ti, round(float(nrm[0]), 4), round(float(nrm[1]), 4), round(float(nrm[2]), 4))
                k = index.get(key)
                if k is None:
                    k = index[key] = len(out_p)
                    out_p.append(pos[vi])
                    out_n.append(nrm)
                    out_uv.append((uv[ti, 0], 1.0 - uv[ti, 1]))
                tri.append(k)
            if tri[0] != tri[1] and tri[1] != tri[2] and tri[0] != tri[2]:
                tris.append(tri)
        shapes.append({
            'name': name.replace('##', '').replace(':', '_'),
            'source': name, 'bone': bone,
            'positions': np.round(np.array(out_p), 5).tolist(),
            'normals': np.round(np.array(out_n), 5).tolist(),
            'uvs': np.round(np.array(out_uv), 6).tolist(),
            'triangles': tris,
            'material': n.get('material', {}).get('name'),
            'maps': [m.get('file') for m in n.get('material', {}).get('maps', [])],
        })
    bones = {}
    for bone in cfg.PARTS:
        if bone == 'Weapon':
            continue
        bones[bone] = {'parent': cfg.FO4_PARENT[bone], 'local': cfg.PARTS[bone][1].tolist(),
                       'in_weapon': cfg.pivot_in_weapon(bone).tolist()}
    allp = np.vstack([np.array(s['positions']) + cfg.pivot_in_weapon(s['bone']) for s in shapes])
    muzzle = None
    for s in shapes:
        if s['source'] == MUZZLE_MESH:
            p = np.array(s['positions'])
            muzzle = [float((p[:, 0].min() + p[:, 0].max()) / 2), float(p[:, 1].max()), float((p[:, 2].min() + p[:, 2].max()) / 2)]
    return {'format': 'port_parts 1', 'units': 'Fallout 4 game units, weapon space: +Y muzzle, +Z up, +X right',
            'T': cfg.T.tolist(), 'R': cfg.R.tolist(), 'bones': bones, 'shapes': shapes,
            'bbox_min': allp.min(0).tolist(), 'bbox_max': allp.max(0).tolist(), 'muzzle': muzzle}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--meshes', required=True)
    ap.add_argument('--rest', required=True)
    ap.add_argument('-o', '--output', required=True)
    a = ap.parse_args()
    data = convert(a.meshes, a.rest)
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    json.dump(data, open(a.output, 'w'), separators=(',', ':'))
    print('T', np.round(cfg.T, 3), 'bbox', np.round(data['bbox_min'], 2), np.round(data['bbox_max'], 2), 'muzzle', np.round(data['muzzle'], 2))
    for s in data['shapes']:
        p = np.array(s['positions'])
        print('%-22s bone=%-22s verts=%5d tris=%5d min=%s max=%s' % (s['name'], s['bone'], len(p), len(s['triangles']), np.round(p.min(0), 2), np.round(p.max(0), 2)))
