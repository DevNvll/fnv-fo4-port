"""Render the retargeted result: vanilla Fallout 4 first-person arms and the converted gun.

blender --background --factory-startup --python render_fo4.py -- \
    --poses out/fo4/poses/SCENE.npz --parts out/fo4mesh/parts.json --out DIR --frames 0,30,60

The arm meshes are the vanilla meshes (esx nif export, game units). Each vertex is
sum(weight * bone world * skin-to-bone * vertex), as in the game.
"""
import argparse
import json
import math
import os
import struct
import sys
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
from render_source import setup_scene, look_at, place_view  # noqa: E402

FO4 = os.environ.get('PORT_RIG') or os.path.join(os.path.dirname(os.path.dirname(HERE)), 'rig')
PART_COLORS = {'nmClip_0': (0.75, 0.45, 0.1), 'nmClip_2': (0.9, 0.75, 0.2), 'nmClip2_0': (0.35, 0.35, 0.8), 'nmClip2_2': (0.9, 0.75, 0.2),
               'nmBar_0': (0.8, 0.2, 0.2), 'MP7ChargingHandel_0': (0.2, 0.7, 0.8), 'Trigger_0': (0.6, 0.2, 0.7)}


def read_glb(path):
    b = open(path, 'rb').read()
    _, _, total = struct.unpack_from('<III', b, 0)
    off = 12
    chunks = []
    while off < total:
        ln, typ = struct.unpack_from('<II', b, off)
        chunks.append(b[off + 8:off + 8 + ln])
        off += 8 + ln
    js = json.loads(chunks[0])
    binb = chunks[1]
    ctype = {5120: 'i1', 5121: 'u1', 5122: '<i2', 5123: '<u2', 5125: '<u4', 5126: '<f4'}
    ncomp = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}

    def acc(i):
        a = js['accessors'][i]
        bv = js['bufferViews'][a['bufferView']]
        dt = np.dtype(ctype[a['componentType']])
        n = ncomp[a['type']]
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        stride = bv.get('byteStride')
        if stride and stride != dt.itemsize * n:
            raw = np.frombuffer(binb, np.uint8, count=stride * a['count'], offset=start).reshape(a['count'], stride)
            arr = raw[:, :dt.itemsize * n].copy().view(dt).reshape(a['count'], n)
        else:
            arr = np.frombuffer(binb, dt, count=a['count'] * n, offset=start).reshape(a['count'], n)
        if a.get('normalized'):
            arr = arr / float(np.iinfo(dt).max)
        return arr.astype(np.float64) if dt.kind == 'f' or a.get('normalized') else arr
    return js, acc


# glTF point (x, y, z) -> game point (x, -z, y)
A = np.array([[1.0, 0, 0, 0], [0, 0, -1.0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]])
AI = np.linalg.inv(A)


class SkinnedMesh:
    def __init__(self, path):
        js, acc = read_glb(path)
        node = [n for n in js['nodes'] if 'mesh' in n and 'skin' in n][0]
        prim = js['meshes'][node['mesh']]['primitives'][0]
        skin = js['skins'][node['skin']]
        self.name = node['name']
        pos = acc(prim['attributes']['POSITION'])
        self.verts = pos @ A[:3, :3].T
        self.tris = acc(prim['indices']).reshape(-1, 3).astype(int)
        self.joints = acc(prim['attributes']['JOINTS_0']).astype(int)
        self.weights = acc(prim['attributes']['WEIGHTS_0']).astype(float)
        self.joint_names = [js['nodes'][i]['name'] for i in skin['joints']]
        ibm = acc(skin['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1)      # column-major -> matrices
        self.inv_bind = np.array([A @ m @ AI for m in ibm])
        # world matrix of the mesh node in the model (normally identity)
        self.bind_world = {n: np.linalg.inv(self.inv_bind[i]) for i, n in enumerate(self.joint_names)}


class Helpers:
    """Bones that the animation does not have: fixed under an animated bone."""

    def __init__(self, names):
        d = json.load(open(os.path.join(FO4, '1st_skeleton_nodes.json')))['data']
        nodes = d['nodes']
        self.local = {}
        self.parent = {}
        world = {}
        byblock = {n['block']: n for n in nodes}
        for n in nodes:
            m = np.eye(4)
            m[:3, :3] = np.array(n['rotation'], float).reshape(3, 3)
            m[:3, 3] = n['translation']
            p = byblock.get(n['parent']) if n.get('parent') is not None else None
            if p is n:
                p = None
            self.local[n['name']] = m
            self.parent[n['name']] = p['name'] if p is not None else None
        self.animated = set(names)

    def rest_world(self, name):
        m = np.eye(4)
        cur = name
        while cur is not None:
            if cur not in self.local:
                return None
            m = self.local[cur] @ m
            cur = self.parent.get(cur)
        return m

    def chain(self, name):
        """(animated ancestor, matrix of the node relative to it) or None."""
        m = np.eye(4)
        cur = name
        seen = 0
        while cur is not None and cur not in self.animated and seen < 50:
            if cur not in self.local:
                return None
            m = self.local[cur] @ m
            cur = self.parent.get(cur)
            seen += 1
        if cur is None or cur not in self.animated:
            return None
        return cur, m


GUESS = {'Chest_Rear_Skin': 'Chest', 'LArm_ShoulderFat_skin': 'LArm_Collarbone', 'RArm_ShoulderFat_skin': 'RArm_Collarbone'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poses', required=True)
    ap.add_argument('--parts', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', default='0')
    ap.add_argument('--views', default='fp,side')
    ap.add_argument('--fov', type=float, default=60.0)
    ap.add_argument('--tag')
    a = ap.parse_args(sys.argv[sys.argv.index('--') + 1:])
    os.makedirs(a.out, exist_ok=True)
    d = np.load(a.poses, allow_pickle=False)
    poses = d['poses']
    names = [str(x) for x in d['names']]
    idx = {n: i for i, n in enumerate(names)}
    parts = json.load(open(a.parts))
    scene, cam = setup_scene()
    helpers = Helpers(names)
    rest = np.load(os.path.join(FO4, 'rest_world.npy')) if os.path.exists(os.path.join(FO4, 'rest_world.npy')) else None

    arms = []
    for f in ('1stpersonmalebody.glb', '1stpersonmalehands.glb'):
        sm = SkinnedMesh(os.path.join(FO4, f))
        mesh = bpy.data.meshes.new(sm.name)
        mesh.from_pydata([tuple(v) for v in sm.verts], [], [tuple(t) for t in sm.tris])
        mesh.update()
        ob = bpy.data.objects.new(sm.name, mesh)
        bpy.context.collection.objects.link(ob)
        mat = bpy.data.materials.new(sm.name)
        mat.diffuse_color = (0.8, 0.58, 0.47, 1) if 'Hands' in sm.name else (0.25, 0.3, 0.45, 1)
        mesh.materials.append(mat)
        for p in mesh.polygons:
            p.use_smooth = True
        # how each joint follows the animation
        rules = []
        for jn in sm.joint_names:
            if jn in idx:
                rules.append((idx[jn], np.eye(4)))
                continue
            ch = helpers.chain(jn)
            if ch is None:
                base = GUESS.get(jn) or jn.replace('_skin', '')
                if base not in idx:
                    raise SystemExit('no rule for the skin bone ' + jn)
                # fixed to the base bone as at the bind of the mesh
                wb = sm.bind_world.get(base)
                if wb is None:
                    wb = helpers.rest_world(base)
                if wb is None:
                    raise SystemExit('no bind matrix for %s (skin bone %s)' % (base, jn))
                rules.append((idx[base], np.linalg.inv(wb) @ sm.bind_world[jn]))
            else:
                rules.append((idx[ch[0]], ch[1]))
        arms.append((sm, ob, rules))

    gun = []
    for s in parts['shapes']:
        mesh = bpy.data.meshes.new(s['name'])
        p = np.array(s['positions'])
        mesh.from_pydata([tuple(v) for v in p], [], [tuple(t) for t in s['triangles']])
        mesh.update()
        ob = bpy.data.objects.new(s['name'], mesh)
        bpy.context.collection.objects.link(ob)
        mat = bpy.data.materials.new(s['name'])
        mat.diffuse_color = (*PART_COLORS.get(s['name'], (0.28, 0.29, 0.31)), 1)
        mesh.materials.append(mat)
        for poly in mesh.polygons:
            poly.use_smooth = True
        gun.append((p, ob, idx[s['bone']]))

    n = poses.shape[0]
    frames = [min(int(x), n - 1) for x in a.frames.split(',')] if a.frames != 'all' else list(range(n))
    tag = a.tag or os.path.splitext(os.path.basename(a.poses))[0]
    for f in frames:
        pw = poses[f]
        for sm, ob, rules in arms:
            out = np.zeros_like(sm.verts)
            mats = [pw[bi] @ extra @ sm.inv_bind[j] for j, (bi, extra) in enumerate(rules)]
            for k in range(sm.joints.shape[1]):
                jj = sm.joints[:, k]
                ww = sm.weights[:, k]
                for j in np.unique(jj):
                    sel = (jj == j) & (ww > 0)
                    if not sel.any():
                        continue
                    m = mats[j]
                    out[sel] += ww[sel, None] * (sm.verts[sel] @ m[:3, :3].T + m[:3, 3])
            ob.data.vertices.foreach_set('co', out.reshape(-1))
            ob.data.update()
        for p, ob, bi in gun:
            m = pw[bi]
            out = p @ m[:3, :3].T + m[:3, 3]
            ob.data.vertices.foreach_set('co', out.reshape(-1))
            ob.data.update()
        camw = pw[idx['Camera']]
        # the place of the FNV weapon bone: the grip
        wm = pw[idx['Weapon']]
        grip = wm[:3, :3] @ np.array(parts['T']) + wm[:3, 3]
        for view in a.views.split(','):
            if place_view(cam, view, grip):
                pass
            elif view == 'fp':
                cam.data.sensor_width = 36
                cam.data.lens = 18.0 / math.tan(math.radians(a.fov) / 2)
                cam.matrix_world = Matrix([list(r) for r in camw])
            elif view == 'side':
                cam.data.lens = 50
                target = grip + np.array([0, -4, -3.0])
                look_at(cam, target + np.array([55, 6, 6.0]), target)
            elif view == 'left':
                cam.data.lens = 50
                target = grip + np.array([0, -4, -3.0])
                look_at(cam, target + np.array([-55, 6, 6.0]), target)
            elif view == 'top':
                cam.data.lens = 50
                target = grip + np.array([0, -4, 0.0])
                look_at(cam, target + np.array([0, -8, 60.0]), target, up=(0, 1, 0))
            scene.render.filepath = os.path.join(a.out, '%s_f%03d_%s.png' % (tag, f, view))
            bpy.ops.render.render(write_still=True)
    print('RENDERED', len(frames), 'frames')


if __name__ == '__main__':
    main()
