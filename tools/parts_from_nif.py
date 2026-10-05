"""Convert the gun meshes of a New Vegas model (.nif) to Fallout 4 weapon space.

    python3 tools/parts_from_nif.py [-o out/fo4mesh/parts.json]

The model is `model` in [source] of the configuration file of the weapon. Each shape has its
vertices in the local space of the FO4 weapon bone that carries it. A shape below a node of
[parts] goes on the bone of that node. [mesh.bones] gives a different bone for a shape, and
`skip` in [mesh] removes a shape. Each other shape is static (bone Weapon).

The output has the format of parts_from_max.py, and for each shape the name of its material.
A material is one texture set of the model. Its name is the name of the diffuse texture.
"""
import argparse
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_config as cfg


def material_name(textures):
    """The name of the texture set: the stem of the diffuse texture, with no _d."""
    if not textures or not textures[0]:
        return None
    stem = os.path.splitext(textures[0].replace('\\', '/').split('/')[-1])[0]
    return stem[:-2] if stem.lower().endswith('_d') else stem


def convert():
    nif = cfg.MODEL
    node_bone = {node: bone for bone, (node, _) in cfg.PARTS.items() if bone != 'Weapon'}
    root = nif.root()
    parent_of = {}
    shapes, materials = [], {}
    muzzle = casing = casing_dir = None
    for b, parent, wr, wt, ws in nif.walk():
        parent_of[b['name']] = parent['name'] if parent is not None else None
        if b['name'] == 'ProjectileNode':
            muzzle = cfg.fnv_to_fo4(wt).tolist()
        if b['name'] == 'ShellCasingNode':
            casing = wt.tolist()
            # The two games eject a casing along the +Z axis of this point. The axes of the
            # node keep their names, and each one goes to Fallout 4 weapon space.
            casing_dir = (cfg.R @ wr).tolist()
        if 'data' not in b:
            continue
        name = b['name']
        if name in cfg.SKIP_MESHES:
            continue
        bone = cfg.MESH_BONE.get(name)
        n = parent_of[name]
        while bone is None and n is not None:
            bone = node_bone.get(n)
            n = parent_of[n]
        bone = bone or 'Weapon'
        g = nif.blocks[b['data']]
        if g['vertices'] is None or g['normals'] is None or not g['uv']:
            raise SystemExit('the shape %s has no vertices, normals or texture coordinates' % name)
        q = g['vertices'] @ wr.T * ws + wt                      # FNV weapon space, at rest
        pos = cfg.to_bone(bone, cfg.fnv_to_fo4(q))
        nrm = cfg.to_bone_direction(bone, g['normals'] @ wr.T @ cfg.R.T)
        ln = np.linalg.norm(nrm, axis=1, keepdims=True)
        nrm = nrm / np.where(ln > 0, ln, 1)
        tex = nif.textures(b)
        mat = material_name(tex)
        if mat is None:
            raise SystemExit('the shape %s has no diffuse texture' % name)
        materials.setdefault(mat, [t.replace('\\', '/') for t in tex])
        shapes.append({
            'name': name.replace('##', '').replace(':', '_'),
            'source': name, 'bone': bone,
            'positions': np.round(pos, 5).tolist(),
            'normals': np.round(nrm, 5).tolist(),
            'uvs': np.round(g['uv'][0], 6).tolist(),
            'triangles': g['triangles'].tolist(),
            'material_name': mat,
        })
    bones = {}
    for bone in cfg.PARTS:
        if bone == 'Weapon':
            continue
        bones[bone] = {'parent': cfg.FO4_PARENT[bone], 'local': cfg.PARTS[bone][1].tolist(),
                       'in_weapon': cfg.pivot_in_weapon(bone).tolist()}
        if bone in cfg.PART_ROTATION:
            bones[bone]['rotation'] = cfg.PART_ROTATION[bone].tolist()

    def in_weapon(s):
        p = np.array(s['positions'])
        if s['bone'] == 'Weapon' or not cfg.turned(s['bone']):
            return p + cfg.pivot_in_weapon(s['bone'])
        m = cfg.rest_in_weapon(s['bone'])
        return p @ m[:3, :3].T + m[:3, 3]
    allp = np.vstack([in_weapon(s) for s in shapes])
    return {'format': 'port_parts 1', 'units': 'Fallout 4 game units, weapon space: +Y muzzle, +Z up, +X right',
            'T': cfg.T.tolist(), 'R': cfg.R.tolist(), 'bones': bones, 'shapes': shapes, 'materials': materials,
            'bbox_min': allp.min(0).tolist(), 'bbox_max': allp.max(0).tolist(), 'muzzle': muzzle, 'casing_fnv': casing, 'casing_rotation': casing_dir}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output', default=os.path.join(cfg.OUT, 'fo4mesh', 'parts.json'))
    a = ap.parse_args()
    data = convert()
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    json.dump(data, open(a.output, 'w'), separators=(',', ':'))
    print('T', np.round(cfg.T, 3), 'bbox', np.round(data['bbox_min'], 2), np.round(data['bbox_max'], 2), 'muzzle', np.round(data['muzzle'], 2))
    for s in data['shapes']:
        p = np.array(s['positions'])
        print('%-22s bone=%-22s verts=%5d tris=%5d min=%s max=%s material=%s' % (
            s['name'], s['bone'], len(p), len(s['triangles']), np.round(p.min(0), 2), np.round(p.max(0), 2), s['material_name']))
