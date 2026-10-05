"""Make the model file of each attachment of the gun from a New Vegas model (.nif).

    python3 tools/attachments.py

New Vegas has one complete model for each set of weapon mods. Fallout 4 has one mesh for each
part, which a connect point of the receiver holds. Each [[attachment]] table of the
configuration file of the weapon gives one part:

    name = "Suppressor"                     the model is meshes/Weapons/WEAPON/WEAPONSuppressor.glb
    model = "{work}/src/.../GunSil.nif"     the New Vegas model that has the part
    shapes = ["Silencer:0"]                 the shapes of that model that are the part
    connect = "Muzzle"                      the part has the point C-Muzzle, and the receiver gets P-Muzzle
    at = "ProjectileNode"                   optional: a node of the base model that gives the place of the point
    projectile = true                       optional: the part has the point P-ProjectileNode at the place of
                                            the node ProjectileNode of its model (a muzzle part)

The program writes each model with its job file, and out/fo4mesh/attachments.json with the
place of each point and the materials. tools/port_glb.py puts the points into the receiver,
and tools/fnv_textures.py converts the textures.
"""
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_config as cfg
import fnvnif
from port_glb import Glb
from parts_from_nif import material_name


def node_place(nif, name):
    for b, parent, wr, wt, ws in nif.walk():
        if b['name'] == name:
            return wt
    raise SystemExit('%s: no node %s' % (nif.path, name))


def main():
    out = {'attachments': [], 'materials': {}}
    for at in cfg.D.get('attachment', []):
        nif = fnvnif.Nif(cfg.path(at['model']))
        point = cfg.fnv_to_fo4(node_place(cfg.MODEL, at['at'])) if at.get('at') else np.zeros(3)
        name = cfg.NAME + at['name']
        g = Glb()
        root = g.node(name)
        used, found, shapes = [], [], []
        for b, parent, wr, wt, ws in nif.walk():
            if 'data' not in b or b['name'] not in at['shapes']:
                continue
            found.append(b['name'])
            d = nif.blocks[b['data']]
            pos = cfg.fnv_to_fo4(d['vertices'] @ wr.T * ws + wt) - point
            nrm = d['normals'] @ wr.T @ cfg.R.T
            ln = np.linalg.norm(nrm, axis=1, keepdims=True)
            nrm = nrm / np.where(ln > 0, ln, 1)
            tex = nif.textures(b)
            mat = material_name(tex)
            if mat is None:
                raise SystemExit('%s: the shape %s has no diffuse texture' % (at['name'], b['name']))
            out['materials'].setdefault(mat, [t.replace('\\', '/') for t in tex])
            if mat not in used:
                used.append(mat)
            m = g.mesh(b['name'], np.round(pos, 5), np.round(nrm, 5), np.round(d['uv'][0], 6), d['triangles'], mat)
            g.node(b['name'], mesh=m, parent=root)
            # in weapon space, for the contact check of the hands
            shapes.append({'name': at['name'] + '_' + b['name'].replace(':', '_'), 'bone': 'Weapon',
                           'positions': np.round(pos + point, 5).tolist(), 'triangles': d['triangles'].tolist()})
        missing = [s for s in at['shapes'] if s not in found]
        if missing:
            raise SystemExit('%s: the model has no shape %s' % (at['name'], ', '.join(missing)))
        if at.get('projectile'):
            g.node('P-ProjectileNode', cfg.fnv_to_fo4(node_place(nif, 'ProjectileNode')) - point, parent=root)
        path = cfg.path('meshes/Weapons/%s/%s.glb' % (cfg.NAME, name))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        g.write(path, root)
        job = ['# The job file of the part %s of the %s. The model is in game units.' % (at['name'], cfg.NAME),
               'scale = 1.0', '', '[materials]']
        job += ['%s = "Materials/Weapons/%s/%s.bgsm"' % (m, cfg.NAME, m) for m in used]
        job += ['', '[connect]', 'children = ["C-%s"]' % at['connect'], '']
        with open(os.path.splitext(path)[0] + '.mesh.toml', 'w') as f:
            f.write('\n'.join(job))
        out['attachments'].append({'name': at['name'], 'connect': at['connect'], 'point': np.round(point, 5).tolist(),
                                   'model': 'Weapons\\%s\\%s.nif' % (cfg.NAME, name), 'materials': used, 'shapes': shapes})
        print('%-12s C-%-10s point %s  shapes %d  materials %s' % (at['name'], at['connect'], np.round(point, 2), len(found), used))
    path = os.path.join(cfg.OUT, 'fo4mesh', 'attachments.json')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(out, open(path, 'w'), separators=(',', ':'))


if __name__ == '__main__':
    main()
