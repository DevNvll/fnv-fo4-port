"""The mesh file of the source, for the renders and the contact check of the source.

    python3 tools/source_meshes.py            write the file of a NIF source and print its name
    python3 tools/source_meshes.py --path     print the name only

A 3ds Max source has the file of tools/maxmesh.py (out/mesh/SCENE/meshes.json). For a NIF
source this program writes out/mesh/source/meshes.json in the same format: each shape of the
gun model, and the first-person arm meshes of the New Vegas rig (rig/fnv_arms.json).
The bake file of tools/kfbake.py has a node for each shape.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_config as cfg


def path():
    if cfg.SOURCE['kind'] == 'nif':
        return os.path.join(cfg.OUT, 'mesh', 'source', 'meshes.json')
    return os.path.join(cfg.OUT, 'mesh', cfg.SOURCE['meshes'], 'meshes.json')


def write():
    nif = cfg.MODEL
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]]
    nodes = []
    for b, parent, wr, wt, ws in nif.walk():
        if 'data' not in b or b['name'] in cfg.SKIP_MESHES:
            continue
        g = nif.blocks[b['data']]
        nodes.append({'name': b['name'], 'object_offset': {'matrix': identity},
                      'mesh': {'verts': g['vertices'].round(5).tolist(), 'faces': g['triangles'].tolist()}})
    nodes += json.load(open(os.path.join(cfg.RIG, 'fnv_arms.json')))['nodes']
    out = path()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({'format': 'source_meshes 1', 'nodes': nodes}, open(out, 'w'), separators=(',', ':'))
    return out


if __name__ == '__main__':
    if '--path' not in sys.argv and cfg.SOURCE['kind'] == 'nif':
        write()
    print(path())
