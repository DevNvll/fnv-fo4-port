"""Make the rig folder from your own game files.

    python3 tools/make_rig.py --data "/path/Fallout 4/Data" --nvcs /path/NVCS/Meshes/Characters/_1stPerson/Skeleton.nif

The rig folder has files of Fallout 4 and of the New Vegas compatibility skeleton (NVCS), so
it is not in the repository. This program takes each file from your installed game and from
the skeleton file of the NVCS mod:

    1st_skeleton.hkx           the Fallout 4 first-person skeleton (Fallout4 - Animations.ba2)
    TEMPLATE/*.hkx             the first-person clips of each template weapon (the same archive)
    1stpersonmalehands.glb, 1stpersonmalebody.glb, 1st_skeleton_nodes.json
                               the arm meshes and the skeleton nodes, for the check renders
                               (Fallout4 - Meshes.ba2)
    TGunReceiver.bgsm          the material that the textures step copies (Fallout4 - Materials.ba2)
    nvcs_1st.json              the New Vegas first-person skeleton: each node, its parent and its matrix

The program `esx` reads the archives and the meshes: ESX_BIN names it (default `esx`).
rig/finger_tips.json (measurements of the two hand meshes) is in the repository.
rig/fnv_arms.json (the New Vegas arm meshes) is optional: only the renders and the contact
count of the source use it.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

ANIMATIONS = 'Fallout4 - Animations.ba2'
MESHES = 'Fallout4 - Meshes.ba2'
MATERIALS = 'Fallout4 - Materials.ba2'


def esx(*args):
    r = subprocess.run([os.environ.get('ESX_BIN', 'esx'), *args, '--format', 'json'], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit('esx %s failed: %s' % (' '.join(args[:2]), (r.stdout or r.stderr)[:500]))
    return r.stdout


def extract(data, archive, pattern, folder):
    """Extract the entries of an archive that match a pattern; return the files."""
    esx('archive', 'extract', os.path.join(data, archive), '-o', folder, '--pattern', pattern, '--overwrite')
    out = []
    for base, _, files in os.walk(folder):
        out += [os.path.join(base, f) for f in files]
    return sorted(out)


def one(data, archive, path, target):
    with tempfile.TemporaryDirectory() as tmp:
        files = [f for f in extract(data, archive, path, tmp) if os.path.basename(f).lower() == os.path.basename(path).lower()]
        if len(files) != 1:
            raise SystemExit('%s: %d files for %s' % (archive, len(files), path))
        shutil.copyfile(files[0], target)


def nvcs_nodes(path):
    """The nodes of the New Vegas skeleton: name, parent and local matrix (16 numbers with 8
    significant digits, the translation last)."""
    import fnvnif
    import numpy as np
    nif = fnvnif.Nif(path)
    nodes = []
    for b, parent, _, _, _ in nif.walk():
        if 'data' in b:
            continue
        # single precision, as the file has it
        m = np.eye(4, dtype=np.float32)
        m[:3, :3] = b['rotation'].astype(np.float32) * np.float32(b['scale'])
        m[:3, 3] = b['translation'].astype(np.float32)
        nodes.append({'name': b['name'], 'parent': parent['name'] if parent is not None else '',
                      'matrix': [float('%.8g' % v) for v in m.T.reshape(-1)]})
    return {'nodes': nodes, 'meshes': []}


def main():
    ap = argparse.ArgumentParser(description='Make the rig folder from your own game files.')
    ap.add_argument('--data', required=True, help='the Data folder of Fallout 4')
    ap.add_argument('--nvcs', required=True, help='Meshes/Characters/_1stPerson/Skeleton.nif of the NVCS mod')
    ap.add_argument('--out', default=os.path.join(ROOT, 'rig'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    one(a.data, ANIMATIONS, 'Meshes/Actors/Character/_1stPerson/CharacterAssets/skeleton.hkx', os.path.join(a.out, '1st_skeleton.hkx'))
    print('1st_skeleton.hkx')
    # the clips of each template, with names in lower case
    for name in sorted(os.listdir(os.path.join(ROOT, 'templates'))):
        if not name.endswith('.toml'):
            continue
        t = tomllib.load(open(os.path.join(ROOT, 'templates', name), 'rb'))
        folder = os.path.join(a.out, t['clips'])
        os.makedirs(folder, exist_ok=True)
        with tempfile.TemporaryDirectory() as tmp:
            files = [f for f in extract(a.data, ANIMATIONS, 'Meshes/Actors/Character/_1stPerson/Animations/%s/*' % t['game_clips'], tmp)
                     if f.lower().endswith('.hkx') and os.path.basename(os.path.dirname(f)).lower() == t['game_clips'].lower()]
            for f in files:
                shutil.copyfile(f, os.path.join(folder, os.path.basename(f).lower()))
        print('%s: %d clips' % (t['clips'], len(files)))
    for path, stem in (('Meshes/Actors/Character/CharacterAssets/1stPersonMaleHands.nif', '1stpersonmalehands'),
                       ('Meshes/Actors/Character/CharacterAssets/1stPersonMaleBody.nif', '1stpersonmalebody')):
        nif = os.path.join(a.out, stem + '.nif')
        one(a.data, MESHES, path, nif)
        esx('nif', 'export', nif, '-o', os.path.join(a.out, stem + '.glb'), '--scale', '1.0', '--overwrite')
        os.remove(nif)
        print(stem + '.glb')
    with tempfile.TemporaryDirectory() as tmp:
        nif = os.path.join(tmp, 'skeleton.nif')
        one(a.data, MESHES, 'Meshes/Actors/Character/_1stPerson/CharacterAssets/skeleton.nif', nif)
        with open(os.path.join(a.out, '1st_skeleton_nodes.json'), 'w') as f:
            f.write(esx('nif', 'shapes', nif))
    print('1st_skeleton_nodes.json')
    one(a.data, MATERIALS, 'Materials/Weapons/CombatShotgun/TGunReceiver.BGSM', os.path.join(a.out, 'TGunReceiver.bgsm'))
    print('TGunReceiver.bgsm')
    with open(os.path.join(a.out, 'nvcs_1st.json'), 'w') as f:
        json.dump(nvcs_nodes(a.nvcs), f, separators=(',', ':'))
    print('nvcs_1st.json')


if __name__ == '__main__':
    main()
