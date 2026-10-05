"""Evaluate decoded Gamebryo transforms without changing their absolute poses."""
from bisect import bisect_right
import math
from mathutils import Euler, Matrix, Quaternion, Vector


def matrix(values):
    return Matrix([values[i:i + 4] for i in range(0, 16, 4)]).transposed()


def trs(position, rotation, scale=1.0):
    if isinstance(scale, (int, float)):
        scale = (scale,) * 3
    return Matrix.LocRotScale(Vector(position), Quaternion(rotation).normalized(), Vector(scale))


def sample(group, time, default, quaternion=False):
    keys = group.get('keys', [])
    if not keys:
        return default
    kind = group['type']
    if kind not in (1, 2, 5):
        raise ValueError(f'Unsupported key interpolation {kind}')
    i = bisect_right([k['time'] for k in keys], time) - 1
    if i < 0:
        return keys[0]['value']
    if i >= len(keys) - 1 or kind == 5:
        return keys[i]['value']
    a, b = keys[i:i + 2]
    u = (time - a['time']) / (b['time'] - a['time'])
    if quaternion:
        # Ni quaternion keys carry no explicit tangents. SLERP is also the
        # interpolation used by the FNV Niftools Blender importer.
        return Quaternion(a['value']).normalized().slerp(Quaternion(b['value']).normalized(), u)
    vector = isinstance(a['value'], list)
    av, bv = (Vector(a['value']), Vector(b['value'])) if vector else (a['value'], b['value'])
    if kind == 1:
        return av * (1 - u) + bv * u
    forward = Vector(a['forward']) if vector else a['forward']
    backward = Vector(b['backward']) if vector else b['backward']
    return ((2 * u**3 - 3 * u**2 + 1) * av + (u**3 - 2 * u**2 + u) * forward
            + (-2 * u**3 + 3 * u**2) * bv + (u**3 - u**2) * backward)


def track_matrix(track, time, rest):
    p, q, s = rest.decompose()
    def valid(v):
        return all(math.isfinite(x) and abs(x) < 1e20 for x in v)
    p0 = track['translation'] if valid(track['translation']) else p
    q0 = track['rotation'] if valid(track['rotation']) and sum(v*v for v in track['rotation']) > 1e-8 else q
    s0 = track['scale'] if math.isfinite(track['scale']) and abs(track['scale']) < 1e20 else s.x
    p = sample(track.get('translations', {}), time, p0)
    if track.get('rotations', {}).get('type') == 4:
        q = Euler([sample(g, time, 0.0) for g in track['euler']], 'XYZ').to_quaternion()
    else:
        q = sample(track.get('rotations', {}), time, q0, quaternion=True)
    s = sample(track.get('scales', {}), time, s0)
    return trs(p, q, s)


class Skeleton:
    def __init__(self, nodes):
        self.parents = {n['name']: n['parent'] for n in nodes}
        self.local = {n['name']: matrix(n['matrix']) for n in nodes}
        self.names = []
        active = set()
        def visit(name):
            if name in self.names:
                return
            if name in active:
                raise ValueError(f'Cyclic source hierarchy at {name}')
            active.add(name)
            parent = self.parents[name]
            if parent:
                visit(parent)
            active.remove(name)
            self.names.append(name)
        for name in self.parents:
            visit(name)
        self.rest = self.worlds(self.local)

    def worlds(self, local):
        world = {}
        for name in self.names:
            parent = self.parents[name]
            world[name] = (world[parent] if parent else Matrix.Identity(4)) @ local[name]
        return world

    def pose(self, clip, time):
        local = {n: m.copy() for n, m in self.local.items()}
        for t in clip['tracks']:
            if 'visibility' in t:
                continue
            name = t['bone']
            if name not in local:
                raise ValueError(f'No source hierarchy for {name}')
            local[name] = track_matrix(t, time, self.local[name])
        return self.worlds(local)


def sample_times(clip, fps):
    start, stop = clip['start'], clip['stop']
    times = {start, stop}
    times.update(start + i / fps for i in range(math.floor((stop - start) * fps) + 1))
    # Some Max exports have 960 translation keys per second. Resample at the
    # requested frame rate, retaining event boundaries and the exact endpoint.
    times.update(e['time'] for e in clip['events'] if start <= e['time'] <= stop)
    # Float32 source timestamps within 0.1 microsecond of a sample are identical
    # for playback; prefer the original timestamp to avoid near-zero intervals.
    result = []
    for t in sorted(times):
        if result and t - result[-1] < 1e-7:
            result[-1] = t
        else:
            result.append(t)
    return result
