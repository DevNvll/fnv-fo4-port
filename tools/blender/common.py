"""Shared helpers for the MP7 retarget scripts (run inside Blender)."""
import json
import math
import os
import sys
from mathutils import Matrix, Quaternion, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
from kf import Skeleton, trs  # noqa: E402
import anim_fo4  # noqa: E402


def max_to_matrix(v):
    """12 numbers of a 3ds Max matrix (three axis rows, translation row) -> Blender Matrix."""
    return Matrix(((v[0], v[3], v[6], v[9]),
                   (v[1], v[4], v[7], v[10]),
                   (v[2], v[5], v[8], v[11]),
                   (0.0, 0.0, 0.0, 1.0)))


def rigid(m):
    """Remove scale and shear."""
    p, q, _ = m.decompose()
    return Matrix.Translation(p) @ q.normalized().to_matrix().to_4x4()


def load_target(path):
    skel = anim_fo4.load_fo4_skeleton(path)
    nodes = []
    for i, name in enumerate(skel.bones):
        p = skel.reference_pose[i]
        rotation = (p.rotation[3], *p.rotation[:3])
        mat = trs(p.translation, rotation, p.scale)
        parent = skel.bones[skel.parents[i]] if skel.parents[i] >= 0 else ''
        nodes.append({'name': name, 'parent': parent, 'matrix': [v for row in mat.transposed() for v in row]})
    return skel, Skeleton(nodes)


def load_source_rest(path):
    data = json.loads(open(path).read())
    return Skeleton(data['nodes'])


class Bake:
    """World matrices of the scene nodes, relative to the node Bip01."""

    def __init__(self, path, root='Bip01'):
        d = json.loads(open(path).read())
        self.data = d
        self.fps = d['fps']
        self.times = d['times']
        self.tps = d['ticks_per_second']
        self.nodes = d['nodes']
        self.root = root
        self.count = len(self.times)
        self.seconds = [(t - self.times[0]) / self.tps for t in self.times]

    def world(self, frame, names=None):
        nodes = self.nodes
        rootinv = max_to_matrix(nodes[self.root]['world'][frame]).inverted()
        out = {}
        for name, n in nodes.items():
            if names is not None and name not in names:
                continue
            w = n['world'][frame]
            if w is None:
                continue
            out[name] = rootinv @ max_to_matrix(w)
        return out


def fo4_local_tracks(skel, target, poses):
    """Per-bone local translation, rotation (x, y, z, w) and scale for each pose."""
    tracks = []
    for name in skel.bones:
        parent = target.parents[name]
        tr, ro, sc = [], [], []
        previous = None
        for pose in poses:
            local = pose[parent].inverted() @ pose[name] if parent else pose[name]
            p, q, s = local.decompose()
            q.normalize()
            if previous is not None and q.dot(previous) < 0:
                q.negate()
            previous = q.copy()
            row = [*p, q.x, q.y, q.z, q.w, *s]
            if not all(math.isfinite(v) for v in row):
                raise ValueError('non-finite transform at ' + name)
            tr.append([p.x, p.y, p.z])
            ro.append([q.x, q.y, q.z, q.w])
            sc.append([s.x, s.y, s.z])
        tracks.append((tr, ro, sc))
    return tracks


def write_hkx(path, skel, tracks, fps, annotations=()):
    n = len(tracks[0][0])
    anim = anim_fo4.AnimationData()
    anim.num_frames = n
    anim.frame_duration = 1.0 / fps
    anim.duration = (n - 1) / fps
    anim.num_tracks = len(skel.bones)
    anim.bone_names = list(skel.bones)
    anim.track_to_bone_indices = list(range(len(skel.bones)))
    anim.original_skeleton_name = skel.name or 'Root'
    anim.max_frames_per_block = 256
    anim.num_blocks = max(1, (n + 255) // 256)
    anim.block_duration = 255.0 / fps
    anim.tracks = [anim_fo4.TrackData(translations=t, rotations=r, scales=s) for t, r, s in tracks]
    anim.annotations = [anim_fo4.Annotation(time=float(t), text=text) for t, text in annotations]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    anim_fo4.write_fo4_animation(path, anim)
    return anim
