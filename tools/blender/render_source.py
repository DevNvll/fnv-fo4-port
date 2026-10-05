"""Render the source scene (FNV arms and the MP7) from the baked node matrices.

blender --background --factory-startup --python render_source.py -- \
    --bake out/anim/SCENE.json --meshes out/mesh/reload_48/meshes.json --out DIR --frames 0,30,60
"""
import argparse
import json
import math
import os
import sys
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

SKINNED = ('Arms:0', 'Arms:1', 'RightHand:0', 'LeftHand:0')
SKIP = ('bodycaps', 'limbcaps', 'meatneck01', 'meathead01')
COLORS = {'Arms:0': (0.72, 0.5, 0.4), 'Arms:1': (0.25, 0.3, 0.45), 'RightHand:0': (0.8, 0.58, 0.47), 'LeftHand:0': (0.8, 0.58, 0.47),
          '##nmClip:0': (0.75, 0.45, 0.1), '##nmClip:2': (0.9, 0.75, 0.2), '##nmClip2:0': (0.35, 0.35, 0.8), '##nmClip2:2': (0.9, 0.75, 0.2),
          '##nmBar:0': (0.8, 0.2, 0.2), '##MP7ChargingHandel:0': (0.2, 0.7, 0.8), '##Trigger:0': (0.6, 0.2, 0.7)}


def m43(v):
    return np.array(v, float).reshape(4, 3)


def mul(a, b):
    r = np.empty((4, 3))
    r[:3] = a[:3] @ b[:3]
    r[3] = a[3] @ b[:3] + b[3]
    return r


def inv(m):
    ri = np.linalg.inv(m[:3])
    r = np.empty((4, 3))
    r[:3] = ri
    r[3] = -(m[3] @ ri)
    return r


def to_blender(m):
    return Matrix(((m[0, 0], m[1, 0], m[2, 0], m[3, 0]), (m[0, 1], m[1, 1], m[2, 1], m[3, 1]),
                   (m[0, 2], m[1, 2], m[2, 2], m[3, 2]), (0, 0, 0, 1)))


class Source:
    def __init__(self, bake_path, mesh_path):
        self.bake = json.load(open(bake_path))
        self.nodes = self.bake['nodes']
        md = json.load(open(mesh_path))
        self.meshes = [n for n in md['nodes'] if n['name'] not in SKIP]
        self.objs = {}
        self.prep = {}

    def node(self, name, f):
        w = self.nodes[name]['world'][f]
        return m43(w) if w is not None else None

    def build(self):
        for n in self.meshes:
            name = n['name']
            me = n['mesh']
            verts = np.array(me['verts'], float)
            faces = me['faces']
            mesh = bpy.data.meshes.new(name)
            mesh.from_pydata([tuple(v) for v in verts], [], [tuple(f) for f in faces])
            mesh.update()
            ob = bpy.data.objects.new(name, mesh)
            bpy.context.collection.objects.link(ob)
            mat = bpy.data.materials.new(name)
            c = COLORS.get(name, (0.28, 0.29, 0.31))
            mat.diffuse_color = (*c, 1)
            mesh.materials.append(mat)
            for p in mesh.polygons:
                p.use_smooth = True
            self.objs[name] = ob
            off = m43(n['object_offset']['matrix'])
            if n.get('skin') and name in SKINNED:
                sk = n['skin']
                base = m43(sk['mesh_init_object_tm'])
                pw = verts @ base[:3] + base[3]
                bones = []
                for b in sk['bones']:
                    bones.append((b['name'], m43(b['inv_init_object_tm'])))
                nb = len(bones)
                w = np.zeros((len(verts), nb))
                for i, lst in enumerate(sk['weights']):
                    tot = sum(x[1] for x in lst) or 1.0
                    for bi, wt in lst:
                        w[i, bi] += wt / tot
                self.prep[name] = ('skin', pw, bones, w)
            else:
                self.prep[name] = ('rigid', verts, off)

    def pose(self, f):
        for name, ob in self.objs.items():
            p = self.prep[name]
            if p[0] == 'skin':
                _, pw, bones, w = p
                out = np.zeros_like(pw)
                for bi, (bname, invinit) in enumerate(bones):
                    col = w[:, bi]
                    if not col.any():
                        continue
                    tm = self.node(bname, f)
                    if tm is None:
                        tm = inv(invinit)
                    m = mul(invinit, tm)
                    out += col[:, None] * (pw @ m[:3] + m[3])
            else:
                _, verts, off = p
                tm = self.node(name, f)
                m = mul(off, tm)
                out = verts @ m[:3] + m[3]
            ob.data.vertices.foreach_set('co', out.reshape(-1))
            ob.data.update()


def setup_scene(res=(960, 540)):
    if os.environ.get('PORT_RES'):
        res = tuple(int(x) for x in os.environ['PORT_RES'].split('x'))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.color_type = 'MATERIAL'
    scene.display.shading.show_cavity = True
    scene.display.shading.show_shadows = False
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.film_transparent = False
    world = bpy.data.worlds.new('w')
    world.color = (0.55, 0.6, 0.65)
    scene.world = world
    cam_data = bpy.data.cameras.new('cam')
    cam = bpy.data.objects.new('cam', cam_data)
    scene.collection.objects.link(cam)
    cam_data.clip_start = 0.5
    cam_data.clip_end = 5000
    scene.camera = cam
    return scene, cam


def look_at(cam, eye, target, up=(0, 0, 1)):
    eye = Vector(eye)
    d = (Vector(target) - eye).normalized()
    q = d.to_track_quat('-Z', 'Y')
    cam.matrix_world = Matrix.Translation(eye) @ q.to_matrix().to_4x4()


def place_view(cam, view, target):
    """view 'o<azimuth>_<elevation>_<distance>[_<lens>][_<dx>_<dy>_<dz>]': a camera on a sphere around the
    target (degrees; azimuth 0 looks from +X, 90 from +Y). Returns False for another name."""
    if not view.startswith('o'):
        return False
    v = [float(x) for x in view[1:].split('_')]
    az, el, dist = math.radians(v[0]), math.radians(v[1]), v[2]
    cam.data.lens = v[3] if len(v) > 3 else 50
    t = np.array(target, float)
    if len(v) > 6:
        t = t + np.array(v[4:7])
    eye = t + dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    look_at(cam, eye, t)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bake', required=True)
    ap.add_argument('--meshes', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', default='0')
    ap.add_argument('--views', default='fp,side')
    ap.add_argument('--fov', type=float, default=60.0)
    a = ap.parse_args(sys.argv[sys.argv.index('--') + 1:])
    os.makedirs(a.out, exist_ok=True)
    scene, cam = setup_scene()
    src = Source(a.bake, a.meshes)
    src.build()
    n = len(src.bake['times'])
    frames = [min(int(x), n - 1) for x in a.frames.split(',')] if a.frames != 'all' else list(range(n))
    tag = os.path.splitext(os.path.basename(a.bake))[0]
    for f in frames:
        src.pose(f)
        camtm = src.node('Camera1st', f)
        wtm = src.node('Weapon', f)
        for view in a.views.split(','):
            if place_view(cam, view, wtm[3]):
                pass
            elif view == 'fp':
                cam.data.sensor_width = 36
                cam.data.lens = 18.0 / math.tan(math.radians(a.fov) / 2)
                cam.matrix_world = to_blender(camtm) @ Matrix.Rotation(math.pi / 2, 4, 'X')
            elif view == 'side':
                cam.data.lens = 50
                target = wtm[3] + np.array([0, -4, -3.0])
                look_at(cam, target + np.array([55, 6, 6.0]), target)
            elif view == 'left':
                cam.data.lens = 50
                target = wtm[3] + np.array([0, -4, -3.0])
                look_at(cam, target + np.array([-55, 6, 6.0]), target)
            elif view == 'top':
                cam.data.lens = 50
                target = wtm[3] + np.array([0, -4, 0.0])
                look_at(cam, target + np.array([0, -8, 60.0]), target, up=(0, 1, 0))
            scene.render.filepath = os.path.join(a.out, '%s_f%03d_%s.png' % (tag, f, view))
            bpy.ops.render.render(write_still=True)
    print('RENDERED', len(frames), 'frames')


if __name__ == '__main__':
    main()
