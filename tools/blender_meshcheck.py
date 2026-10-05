"""Build the meshes of a meshes.json file in Blender and render check pictures.

Run (no window opens, no user setting changes):

    U=$PORT_WORK/blender-user
    BLENDER_USER_CONFIG=$U/config BLENDER_USER_SCRIPTS=$U/scripts BLENDER_USER_DATAFILES=$U/datafiles \\
    blender --background --factory-startup --python tools/blender_meshcheck.py -- \\
        out/mesh/reload_48/meshes.json out/mesh/reload_48/pictures

Pictures (PNG):
    weapon_side_right.png   the weapon parts, one colour for each part, seen from +Z (the right side of the gun)
    weapon_side_left.png    seen from -Z
    weapon_top.png          seen from +Y (from above), the muzzle points to the right
    weapon_three_quarter.png
    weapon_views.png        the four pictures above in one picture
    weapon_normals.png      left: stored normals, right: normals from the smoothing groups
    weapon_uvgrid.png       the weapon parts with a colour grid texture (UV check)
    uv_weapon.png           UV layout of all weapon parts (they share one texture)
    uv_<node>.png           UV layout of one node
    arm_<node>.png          one skinned mesh in object space: front (seen from +Y), side (from +X), top, three-quarter
    arms_bind.png           the skinned meshes in the bind pose (world space), colour = bone with the largest weight:
                            front, back, top, three-quarter
    arms_bend.png           the same meshes after a test pose (elbows and fingers bent) with the skin weights
    hands_bind.png, hands_bend.png   close pictures of the two hand meshes for the two poses

The data keeps the axes of the scene file. Weapon space: +X is the muzzle direction, +Y is up.
World space of the body: +Z is up, the body looks to +Y.
"""
import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector

SKIP_BODY = ('bodycaps', 'limbcaps', 'meatneck01', 'meathead01')
PALETTE = [(0.80, 0.25, 0.20), (0.20, 0.55, 0.85), (0.25, 0.70, 0.30), (0.90, 0.70, 0.15),
           (0.60, 0.35, 0.80), (0.95, 0.50, 0.15), (0.15, 0.75, 0.75), (0.85, 0.35, 0.60),
           (0.55, 0.55, 0.20), (0.40, 0.40, 0.90), (0.70, 0.85, 0.30), (0.50, 0.30, 0.20),
           (0.30, 0.50, 0.45), (0.90, 0.60, 0.70), (0.45, 0.65, 0.95), (0.75, 0.75, 0.75)]


# ---------------------------------------------------------------- scene helpers

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = 'CYCLES'
    sc.cycles.device = 'CPU'
    sc.cycles.samples = 24
    sc.cycles.use_denoising = False
    sc.render.image_settings.file_format = 'PNG'
    sc.view_settings.view_transform = 'Standard'
    world = bpy.data.worlds.new('world')
    world.use_nodes = True
    bg = world.node_tree.nodes['Background']
    bg.inputs[0].default_value = (0.92, 0.92, 0.92, 1.0)
    bg.inputs[1].default_value = 0.35
    sc.world = world
    cam = bpy.data.cameras.new('cam')
    cam.type = 'ORTHO'
    camo = bpy.data.objects.new('cam', cam)
    sc.collection.objects.link(camo)
    sc.camera = camo
    lights = []
    for name, energy in (('key', 2.2), ('fill', 0.7)):
        ld = bpy.data.lights.new(name, 'SUN')
        ld.energy = energy
        ld.angle = math.radians(8)
        lo = bpy.data.objects.new(name, ld)
        sc.collection.objects.link(lo)
        lights.append(lo)
    return sc, camo, lights


def look_matrix(direction, up):
    """World matrix of an object that looks along `direction` (its local -Z)."""
    d = Vector(direction).normalized()
    z = -d
    x = Vector(up).cross(z)
    if x.length < 1e-6:
        x = Vector((1, 0, 0)).cross(z)
    x.normalize()
    y = z.cross(x)
    return Matrix((x, y, z)).transposed().to_4x4(), x, y, d


def render_view(sc, camo, lights, objs, direction, up, path, res=(1000, 700), margin=1.12):
    """Orthographic picture of `objs` (all other mesh objects are hidden)."""
    for o in bpy.data.objects:
        if o.type == 'MESH':
            o.hide_render = o not in objs
    pts = []
    for o in objs:
        mw = o.matrix_world
        pts.extend(mw @ v.co for v in o.data.vertices)
    pts = np.array([tuple(p) for p in pts])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    center = Vector((lo + hi) / 2)
    radius = float(np.linalg.norm(hi - lo)) / 2 + 1e-6
    m, x, y, d = look_matrix(direction, up)
    px = pts @ np.array(x)
    py = pts @ np.array(y)
    ex, ey = px.max() - px.min(), py.max() - py.min()
    cx = (px.max() + px.min()) / 2 - center.dot(x)
    cy = (py.max() + py.min()) / 2 - center.dot(y)
    aspect = res[0] / res[1]
    camo.data.ortho_scale = max(ex, ey * aspect) * margin
    camo.data.clip_start = 0.01
    camo.data.clip_end = radius * 40
    m.translation = center + x * cx + y * cy - d * (radius * 10)
    camo.matrix_world = m
    for lo_, (fx, fy, fz) in zip(lights, ((0.7, -0.9, 1.0), (-0.9, -0.1, 1.0))):
        ldir = d * fz + x * fx + y * fy
        lm, _, _, _ = look_matrix(ldir, y)
        lo_.matrix_world = lm
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    return path


def load_pixels(path):
    img = bpy.data.images.load(path)
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    bpy.data.images.remove(img)
    return px.reshape(h, w, 4)


def save_pixels(px, path):
    h, w = px.shape[:2]
    img = bpy.data.images.new('out', w, h, alpha=False)
    img.pixels.foreach_set(np.ascontiguousarray(px, np.float32).ravel())
    img.filepath_raw = path
    img.file_format = 'PNG'
    img.save()
    bpy.data.images.remove(img)


def contact_sheet(paths, out, cols=2, remove=False):
    """Put pictures of the same size into one picture. The first picture is at the top left."""
    tiles = [load_pixels(p) for p in paths]
    h, w = tiles[0].shape[:2]
    rows = (len(tiles) + cols - 1) // cols
    sheet = np.ones((rows * h, cols * w, 4), np.float32)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        y0 = (rows - 1 - r) * h                 # pixel rows start at the bottom
        sheet[y0:y0 + h, c * w:(c + 1) * w] = t
        sheet[y0:y0 + h, c * w:c * w + 2] = (0.3, 0.3, 0.3, 1)
        sheet[y0:y0 + 2, c * w:(c + 1) * w] = (0.3, 0.3, 0.3, 1)
    save_pixels(sheet, out)
    if remove:
        for p in paths:
            os.remove(p)
    return out


# ---------------------------------------------------------------- mesh helpers

def mat4(m43):
    """Matrix of the JSON file (row vectors, 4 rows of 3) as a Blender matrix (column vectors)."""
    a = np.array(m43, float)
    m = np.eye(4)
    m[:3, :3] = a[:3].T
    m[:3, 3] = a[3]
    return Matrix(m.tolist())


def make_material(name, color=None, image=None, attribute=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = nt.nodes['Principled BSDF']
    bsdf.inputs['Roughness'].default_value = 0.6
    if color is not None:
        bsdf.inputs['Base Color'].default_value = (*color, 1.0)
    if image is not None:
        tex = nt.nodes.new('ShaderNodeTexImage')
        tex.image = image
        nt.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    if attribute is not None:
        at = nt.nodes.new('ShaderNodeAttribute')
        at.attribute_name = attribute
        nt.links.new(at.outputs['Color'], bsdf.inputs['Base Color'])
    return mat


def make_object(sc, name, mesh, normals='spec', verts=None):
    """A Blender mesh object from one `mesh` entry of the JSON file.

    normals: 'spec' uses the stored normals, 'smooth' uses the normals from the
    smoothing groups, None lets Blender use flat faces.
    """
    v = np.array(mesh['verts'] if verts is None else verts, float)
    f = np.array(mesh['faces'], int)
    me = bpy.data.meshes.new(name)
    me.from_pydata(v.tolist(), [], f.tolist())
    me.update()
    assert len(me.polygons) == len(f) and len(me.loops) == len(f) * 3, name
    if mesh.get('uv') is not None:
        uv = np.array(mesh['uv'], float)[np.array(mesh['uv_faces'], int)].reshape(-1, 2)
        layer = me.uv_layers.new(name='UVMap')
        layer.data.foreach_set('uv', uv.ravel())
    loop_normals = None
    if normals == 'spec' and mesh.get('spec_normals') is not None:
        loop_normals = np.array(mesh['spec_normals'], float)[np.array(mesh['spec_normal_faces'], int)].reshape(-1, 3)
    elif normals == 'smooth':
        loop_normals = np.array(mesh['normals'], float).reshape(-1, 3)
    if loop_normals is not None:
        me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
        me.normals_split_custom_set(loop_normals.tolist())
    ob = bpy.data.objects.new(name, me)
    sc.collection.objects.link(ob)
    return ob


def set_corner_colors(ob, vert_colors, faces):
    me = ob.data
    attr = me.color_attributes.new(name='Col', type='FLOAT_COLOR', domain='CORNER')
    col = np.ones((len(faces) * 3, 4), np.float32)
    col[:, :3] = np.array(vert_colors, np.float32)[np.array(faces, int).reshape(-1)]
    attr.data.foreach_set('color', col.ravel())


def uv_layout(meshes, path, size=1024):
    """Draw the triangles of map channel 1 into a picture. (0, 0) is at the bottom left."""
    img = np.ones((size, size, 4), np.float32)
    shades = [(0.05, 0.05, 0.05), (0.75, 0.1, 0.1), (0.1, 0.35, 0.8), (0.1, 0.55, 0.15), (0.6, 0.4, 0.0),
              (0.5, 0.1, 0.6), (0.0, 0.5, 0.5), (0.4, 0.4, 0.4)]
    for k, mesh in enumerate(meshes):
        if mesh.get('uv') is None:
            continue
        uv = np.array(mesh['uv'], float)
        tf = np.array(mesh['uv_faces'], int)
        color = shades[k % len(shades)] if len(meshes) > 1 else shades[0]
        for a, b in ((0, 1), (1, 2), (2, 0)):
            p0 = uv[tf[:, a]] * (size - 1)
            p1 = uv[tf[:, b]] * (size - 1)
            n = int(np.ceil(np.abs(p1 - p0).max())) + 2
            t = np.linspace(0.0, 1.0, min(n, 4 * size))[None, :, None]
            pts = p0[:, None, :] * (1 - t) + p1[:, None, :] * t
            xi = np.round(pts[..., 0]).astype(int).ravel()
            yi = np.round(pts[..., 1]).astype(int).ravel()
            ok = (xi >= 0) & (xi < size) & (yi >= 0) & (yi < size)
            img[yi[ok], xi[ok], :3] = color
    img[0, :, :3] = img[-1, :, :3] = img[:, 0, :3] = img[:, -1, :3] = (0.6, 0.6, 0.6)
    save_pixels(img, path)
    return path


# ---------------------------------------------------------------- skin helpers

def to_np(m43):
    return np.array(m43, float)


def xform(p, m):
    return p @ m[:3] + m[3]


def inv43(m):
    ri = np.linalg.inv(m[:3])
    return np.vstack([ri, -(m[3] @ ri)])


def mul43(a, b):
    return np.vstack([a[:3] @ b[:3], a[3] @ b[:3] + b[3]])


def rot_local(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4)[:, :3].copy()
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    m[i, i] = c
    m[i, j] = s
    m[j, i] = -s
    m[j, j] = c
    return m


def posed_bones(skin, pose):
    """World matrices of the bones of one skin after a test pose.

    pose: {bone name: (axis, angle)}: a rotation in the space of the bone. The pose
    of the bone B is the world transform D_B = inverse(init_B) * R * init_B. The
    bone B and each bone below it get D_B; with more than one posed bone in the
    chain the transforms follow each other from the tip to the root.
    """
    out = {}
    for b in skin['bones']:
        tm = to_np(b['init_node_tm'])
        chain = ([b['name']] if b['name'] in pose else []) + [a for a in b['ancestors'] if a in pose]
        for a in chain:
            if a not in pose_init:
                continue
            ia = pose_init[a]
            tm = mul43(tm, mul43(mul43(inv43(ia), rot_local(*pose[a])), ia))
        out[b['name']] = tm
    return out


pose_init = {}


def deform(node, pose):
    """Skin the vertices of a node with the test pose. Returns the points in world space."""
    skin = node['skin']
    v = np.array(node['mesh']['verts'], float)
    base = to_np(skin['mesh_init_object_tm'])
    vw = xform(v, base)
    new = posed_bones(skin, pose)
    out = np.zeros_like(vw)
    wsum = np.zeros(len(vw))
    for i, ws in enumerate(skin['weights']):
        for bi, w in ws:
            b = skin['bones'][bi]
            p = xform(xform(vw[i], to_np(b['inv_init_object_tm'])), new[b['name']])
            out[i] += w * p
            wsum[i] += w
    return out / np.maximum(wsum, 1e-9)[:, None]


# ---------------------------------------------------------------- main

def main():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    if len(argv) < 2:
        print('use: blender --background --factory-startup --python blender_meshcheck.py -- meshes.json OUT_FOLDER')
        sys.exit(2)
    data = json.load(open(argv[0]))
    out = argv[1]
    os.makedirs(out, exist_ok=True)
    nodes = data['nodes']
    weapon = [n for n in nodes if 'skin' not in n]
    body = [n for n in nodes if 'skin' in n and n['name'] not in SKIP_BODY]

    def P(name):
        return os.path.join(out, name)

    def safe(name):
        return ''.join(ch if ch.isalnum() else '_' for ch in name).strip('_')

    # ---- (a) the weapon parts, each in its object space with its offset transform only
    # A node with the same mesh at the same place as an earlier node is not drawn
    # (two equal surfaces at one place give noise in the picture).
    seen = {}
    shown = []
    for n in weapon:
        key = (json.dumps(n['mesh']['verts']), json.dumps(n['mesh']['faces']), json.dumps(n['object_offset']['matrix']))
        if key in seen:
            print('not drawn: %s has the same mesh at the same place as %s' % (n['name'], seen[key]))
            continue
        seen[key] = n['name']
        shown.append(n)
    weapon = shown
    sc, cam, lights = reset_scene()
    objs = []
    print('weapon parts and their colours (red, green, blue):')
    for i, n in enumerate(weapon):
        ob = make_object(sc, n['name'], n['mesh'], 'spec')
        ob.matrix_world = mat4(n['object_offset']['matrix'])
        col = PALETTE[i % len(PALETTE)]
        ob.data.materials.append(make_material(n['name'], color=col))
        objs.append(ob)
        print('  %-24s %s' % (n['name'], col))
    if objs:
        views = [('side_right', (0, 0, -1), (0, 1, 0)), ('side_left', (0, 0, 1), (0, 1, 0)),
                 ('top', (0, -1, 0), (0, 0, -1)), ('three_quarter', (-0.55, -0.45, -0.70), (0, 1, 0))]
        tiles = [render_view(sc, cam, lights, objs, d, u, P('weapon_%s.png' % nm)) for nm, d, u in views]
        contact_sheet(tiles, P('weapon_views.png'))

        # stored normals against smoothing group normals, one grey material
        grey = make_material('grey', color=(0.6, 0.6, 0.62))
        for ob in objs:
            ob.data.materials.clear()
            ob.data.materials.append(grey)
        t1 = render_view(sc, cam, lights, objs, (-0.55, -0.45, -0.70), (0, 1, 0), P('_n_spec.png'))
        objs2 = []
        for n in weapon:
            ob = make_object(sc, n['name'] + '_sg', n['mesh'], 'smooth')
            ob.matrix_world = mat4(n['object_offset']['matrix'])
            ob.data.materials.append(grey)
            objs2.append(ob)
        t2 = render_view(sc, cam, lights, objs2, (-0.55, -0.45, -0.70), (0, 1, 0), P('_n_smooth.png'))
        contact_sheet([t1, t2], P('weapon_normals.png'), remove=True)
        for ob in objs2:
            bpy.data.objects.remove(ob)

        # UV check with a colour grid
        grid = bpy.data.images.new('grid', 2048, 2048)
        grid.generated_type = 'COLOR_GRID'
        gm = make_material('grid', image=grid)
        for ob in objs:
            ob.data.materials.clear()
            ob.data.materials.append(gm)
        tiles = [render_view(sc, cam, lights, objs, d, u, P('_g_%s.png' % nm))
                 for nm, d, u in (('a', (0, 0, -1), (0, 1, 0)), ('b', (-0.55, -0.45, -0.70), (0, 1, 0)))]
        contact_sheet(tiles, P('weapon_uvgrid.png'), cols=1, remove=True)
        uv_layout([n['mesh'] for n in weapon], P('uv_weapon.png'))
        for n in weapon:
            if n['name'] in ('Receiver:0', '##nmClip:0', 'Stock:0', 'IronSights:0'):
                uv_layout([n['mesh']], P('uv_%s.png' % safe(n['name'])))

    # ---- (b) each skinned mesh in its object space
    for n in body:
        sc, cam, lights = reset_scene()
        ob = make_object(sc, n['name'], n['mesh'], 'spec')
        ob.matrix_world = mat4(n['object_offset']['matrix'])
        ob.data.materials.append(make_material('skin', color=(0.78, 0.62, 0.52)))
        v = np.array(n['mesh']['verts'], float)
        ext = v.max(axis=0) - v.min(axis=0)
        # Object space of these meshes: Z is the long axis of the body. The hands are turned by their bind matrix.
        views = [('front', (0, -1, 0), (0, 0, 1)), ('side', (-1, 0, 0), (0, 0, 1)),
                 ('top', (0, 0, -1), (0, 1, 0)), ('q', (-0.6, -0.65, -0.45), (0, 0, 1))]
        tiles = [render_view(sc, cam, lights, [ob], d, u, P('_b_%s.png' % nm), res=(800, 700)) for nm, d, u in views]
        contact_sheet(tiles, P('arm_%s.png' % safe(n['name'])), remove=True)
        uv_layout([n['mesh']], P('uv_%s.png' % safe(n['name'])))
        print('%s: object space size %s' % (n['name'], np.round(ext, 2).tolist()))

    # ---- skin check: bind pose in world space, colour = bone with the largest weight; then a test pose
    if body:
        names = sorted({b['name'] for n in body for b in n['skin']['bones']})
        rng = np.random.RandomState(7)
        bone_col = {nm: tuple(rng.uniform(0.15, 0.95, 3)) for nm in names}
        for n in body:
            for b in n['skin']['bones']:
                pose_init.setdefault(b['name'], to_np(b['init_node_tm']))
        pose = {}
        for side in ('L', 'R'):
            pose['Bip01 %s Forearm' % side] = (2, math.radians(-70))
            for fgr in ('1', '2', '3', '4'):
                pose['Bip01 %s Finger%s' % (side, fgr)] = (2, math.radians(-45))
                pose['Bip01 %s Finger%s1' % (side, fgr)] = (2, math.radians(-60))
                pose['Bip01 %s Finger%s2' % (side, fgr)] = (2, math.radians(-40))
        for label, use_pose in (('bind', None), ('bend', pose)):
            sc, cam, lights = reset_scene()
            mat = make_material('bones', attribute='Col')
            objs = []
            for n in body:
                skin = n['skin']
                base = to_np(skin['mesh_init_object_tm'])
                v = np.array(n['mesh']['verts'], float)
                vw = xform(v, base) if use_pose is None else deform(n, use_pose)
                # The stored normals are in object space; turn them with the bind matrix for the picture.
                mesh = dict(n['mesh'])
                if use_pose is None and mesh.get('spec_normals') is not None:
                    mesh['spec_normals'] = (np.array(mesh['spec_normals'], float) @ base[:3]).tolist()
                ob = make_object(sc, n['name'], mesh, 'spec' if use_pose is None else None, verts=vw)
                if use_pose is not None:
                    ob.data.polygons.foreach_set('use_smooth', [True] * len(ob.data.polygons))
                cols = []
                for ws in skin['weights']:
                    bi = max(ws, key=lambda x: x[1])[0]
                    cols.append(bone_col[skin['bones'][bi]['name']])
                set_corner_colors(ob, cols, n['mesh']['faces'])
                ob.data.materials.append(mat)
                objs.append(ob)
            views = [('front', (0, -1, 0), (0, 0, 1)), ('back', (0, 1, 0), (0, 0, 1)),
                     ('top', (0, 0, -1), (0, 1, 0)), ('q', (-0.6, -0.65, -0.45), (0, 0, 1))]
            tiles = [render_view(sc, cam, lights, objs, d, u, P('_s_%s.png' % nm), res=(900, 800)) for nm, d, u in views]
            contact_sheet(tiles, P('arms_%s.png' % label), remove=True)
            # close views of the hand meshes: from above and from the front
            hands = [o for o in objs if 'Hand' in o.name]
            if hands:
                tiles = [render_view(sc, cam, lights, [h], d, u, P('_h_%s_%d.png' % (safe(h.name), k)), res=(700, 600))
                         for h in hands for k, (d, u) in enumerate((((0, 0, -1), (0, 1, 0)), ((0, -1, 0), (0, 0, 1))))]
                contact_sheet(tiles, P('hands_%s.png' % label), remove=True)
    print('pictures are in', out)


main()
