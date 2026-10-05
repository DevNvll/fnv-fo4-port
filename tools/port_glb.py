"""Write the model file (glTF 2.0, .glb) of the gun for the `meshes/` folder of an esx project.

    python3 tools/port_glb.py --parts out/fo4mesh/parts.json [-o PROJECT/meshes/Weapons/NAME/NAMEReceiver.glb]

The output, the name of the static node and the materials are in [glb] and [materials] of the
configuration file of the weapon. The job file (.mesh.toml) is written beside the model.

The model is in game units (the job file has scale = 1.0). The game point (x, y, z) is the
model point (x, z, -y). The node names are the names of the Fallout 4 weapon bones, so the
animation moves them.
"""
import argparse
import json
import os
import struct
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def g2m(p):
    p = np.asarray(p, np.float64)
    return np.stack([p[..., 0], p[..., 2], -p[..., 1]], axis=-1)


class Glb:
    def __init__(self):
        self.bin = bytearray()
        self.views = []
        self.accessors = []
        self.meshes = []
        self.nodes = []
        self.materials = []
        self.matidx = {}

    def _view(self, data, target=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        off = len(self.bin)
        self.bin.extend(data)
        v = {'buffer': 0, 'byteOffset': off, 'byteLength': len(data)}
        if target:
            v['target'] = target
        self.views.append(v)
        return len(self.views) - 1

    def _acc(self, arr, ctype, atype, target=None, minmax=False):
        a = {'bufferView': self._view(arr.tobytes(), target), 'componentType': ctype, 'count': int(arr.shape[0]), 'type': atype}
        if minmax:
            a['min'] = [float(x) for x in arr.min(0)]
            a['max'] = [float(x) for x in arr.max(0)]
        self.accessors.append(a)
        return len(self.accessors) - 1

    def material(self, name):
        if name not in self.matidx:
            self.matidx[name] = len(self.materials)
            self.materials.append({'name': name, 'pbrMetallicRoughness': {'baseColorFactor': [0.3, 0.3, 0.32, 1.0], 'metallicFactor': 0.4, 'roughnessFactor': 0.6}})
        return self.matidx[name]

    def mesh(self, name, pos, nrm, uv, tris, material):
        pos = g2m(pos).astype(np.float32)
        nrm = g2m(nrm).astype(np.float32)
        uv = np.asarray(uv, np.float32)
        tris = np.asarray(tris)
        idx = tris.reshape(-1).astype(np.uint16 if len(pos) < 65536 else np.uint32)
        prim = {'attributes': {'POSITION': self._acc(pos, 5126, 'VEC3', 34962, True),
                               'NORMAL': self._acc(nrm, 5126, 'VEC3', 34962),
                               'TEXCOORD_0': self._acc(uv, 5126, 'VEC2', 34962)},
                'indices': self._acc(idx, 5123 if idx.dtype == np.uint16 else 5125, 'SCALAR', 34963),
                'material': self.material(material), 'mode': 4}
        self.meshes.append({'name': name, 'primitives': [prim]})
        return len(self.meshes) - 1

    def node(self, name, translation=None, mesh=None, parent=None, rotation=None):
        n = {'name': name}
        if translation is not None and np.any(np.abs(translation) > 0):
            n['translation'] = [float(x) for x in g2m(translation)]
        if rotation is not None:
            # a game rotation (x, y, z, w) in the axes of the model: the axis turns as a point
            x, y, z, w = [float(v) for v in rotation]
            n['rotation'] = [x, z, -y, w]
        if mesh is not None:
            n['mesh'] = mesh
        self.nodes.append(n)
        i = len(self.nodes) - 1
        if parent is not None:
            self.nodes[parent].setdefault('children', []).append(i)
        return i

    def write(self, path, root):
        js = {'asset': {'version': '2.0', 'generator': 'port_glb.py'}, 'scene': 0, 'scenes': [{'nodes': [root]}],
              'nodes': self.nodes, 'meshes': self.meshes, 'materials': self.materials, 'accessors': self.accessors,
              'bufferViews': self.views, 'buffers': [{'byteLength': len(self.bin)}]}
        jb = json.dumps(js, separators=(',', ':')).encode()
        while len(jb) % 4:
            jb += b' '
        bb = bytes(self.bin)
        while len(bb) % 4:
            bb += b'\0'
        total = 12 + 8 + len(jb) + 8 + len(bb)
        with open(path, 'wb') as f:
            f.write(struct.pack('<III', 0x46546C67, 2, total))
            f.write(struct.pack('<II', len(jb), 0x4E4F534A))
            f.write(jb)
            f.write(struct.pack('<II', len(bb), 0x004E4942))
            f.write(bb)


def matrix_quat(r):
    """3x3 rotation matrix (column vectors) -> (w, x, y, z)."""
    r = np.asarray(r, float)
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q = q / np.linalg.norm(q)
    return (q if q[0] >= 0 else -q).tolist()


def build(parts, out, material, static_name, casing=None, materials=None, name='gun', casing_rotation=None, points=()):
    g = Glb()
    root = g.node('WEAPON')
    bone_nodes = {'Weapon': root}
    for b, info in parts['bones'].items():
        bone_nodes[b] = g.node(b, np.array(info['local']), parent=bone_nodes[info['parent']], rotation=info.get('rotation'))
    static = g.node(static_name, parent=root)
    used = []
    for s in parts['shapes']:
        mat = s.get('material_name') or material
        if mat not in used:
            used.append(mat)
        m = g.mesh(s['source'].replace('##', ''), s['positions'], s['normals'], s['uvs'], s['triangles'], mat)
        parent = static if s['bone'] == 'Weapon' else bone_nodes[s['bone']]
        g.node(s['source'].replace('##', ''), mesh=m, parent=parent)
    mz = np.array(parts['muzzle'])
    g.node('ProjectileNode', mz, parent=static)
    # a node with no mesh and no child whose name starts with P- is a parent connect point
    done = []
    for point_name, place in points:
        if point_name not in done:
            done.append(point_name)
            g.node('P-' + point_name, np.array(place, float), parent=static)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    g.write(out, root)
    job = ['# The job file of the %s receiver model. The model is in game units.' % name,
           'scale = 1.0', '', '[materials]']
    job += ['%s = "%s"' % (m, (materials or {})[m]) for m in used]
    job += ['', '[connect]', 'children = ["C-Receiver"]', '']
    if casing is not None:
        job += ['[[connect.parent]]', 'name = "P-Casing"', 'parent = "%s"' % static_name,
                'translation = [%.4f, %.4f, %.4f]' % tuple(casing), casing_rotation or 'rotation = [0.9396926, 0.0, -0.3420201, 0.0]', 'scale = 1.0', '']
    with open(os.path.splitext(out)[0] + '.mesh.toml', 'w') as f:
        f.write('\n'.join(job))
    return g


if __name__ == '__main__':
    import port_config as cfg
    ap = argparse.ArgumentParser()
    ap.add_argument('--parts', default=os.path.join(cfg.OUT, 'fo4mesh', 'parts.json'))
    ap.add_argument('-o', '--output', default=cfg.path(cfg.D['glb']['output']))
    a = ap.parse_args()
    parts = json.load(open(a.parts))
    # the ejection port, in FNV weapon space (the node ShellCasingNode of the source)
    casing = cfg.D['weapon'].get('casing', parts.get('casing_fnv'))
    if casing is not None:
        casing = cfg.fnv_to_fo4(np.array(casing, float))
    materials = {m: 'Materials/Weapons/%s/%s.bgsm' % (cfg.NAME, m) for m in parts.get('materials', {})}
    materials.update(cfg.D.get('materials', {}))
    # The game ejects a casing along the +Z axis of the point P-Casing (the vanilla 10mm pistol
    # and the combat rifle eject to the right and up with it). With no rotation from the source,
    # the rotation is that of the vanilla submachine gun.
    rotation = None
    if parts.get('casing_rotation') is not None:
        rotation = 'rotation = [%.7f, %.7f, %.7f, %.7f]' % tuple(matrix_quat(parts['casing_rotation']))
    points = []
    if cfg.D.get('attachment'):
        attachments = json.load(open(os.path.join(cfg.OUT, 'fo4mesh', 'attachments.json')))
        points = [(x['connect'], x['point']) for x in attachments['attachments']]
    g = build(parts, a.output, cfg.D['glb'].get('material'), cfg.D['glb']['static_name'], casing=casing,
              materials=materials, name=cfg.NAME, casing_rotation=rotation, points=points)
    print('nodes', len(g.nodes), 'meshes', len(g.meshes), 'bytes', len(g.bin),
          'casing', None if casing is None else np.round(casing, 3), 'muzzle', np.round(parts['muzzle'], 3))
