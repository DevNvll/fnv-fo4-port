"""Convert the textures of a New Vegas model to Fallout 4 textures and material files.

    python3 tools/fnv_textures.py [--parts out/fo4mesh/parts.json]

For each material of the parts file and of the attachments file (one texture set of a model):

    NAME_d.dds   the diffuse texture
    NAME_n.dds   the red and green channels of the New Vegas normal map
    NAME_s.dds   specular in red and glossiness in green, from the alpha channel of the
                 New Vegas normal map (its specular mask)
    NAME.bgsm    a copy of the material of the template weapon with these three textures

The files go to assets/Textures/Weapons/WEAPON and assets/Materials/Weapons/WEAPON of the
project. A texture set whose files are not there gets one color, and the step says so.
[textures.stand_in] gives the color (material name = [r, g, b], or `default`; without it a
dark gray).
[textures] of the configuration file: max_size (larger textures are made smaller),
specular (the factor of the specular mask), gloss_base and gloss_range (glossiness = base +
range * mask). The program `esx` writes the DDS files: ESX_BIN names it.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_config as cfg

Image.MAX_IMAGE_PIXELS = None
ESX = os.environ.get('ESX_BIN', 'esx')


def find(root, relative):
    """A file below root by its path with no regard to case. The path can start with textures/."""
    parts = [p for p in relative.replace('\\', '/').split('/') if p]
    if parts and parts[0].lower() == 'textures':
        parts = parts[1:]
    here = root
    for p in parts:
        names = {n.lower(): n for n in os.listdir(here)} if os.path.isdir(here) else {}
        if p.lower() not in names:
            return None
        here = os.path.join(here, names[p.lower()])
    return here


def esx(*args):
    r = subprocess.run([ESX, *args, '--format', 'json'], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit('esx %s failed: %s' % (' '.join(args[:2]), (r.stdout or r.stderr)[:600]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--parts', default=os.path.join(cfg.OUT, 'fo4mesh', 'parts.json'))
    a = ap.parse_args()
    opt = cfg.D.get('textures', {})
    max_size = int(opt.get('max_size', 2048))
    spec_factor = float(opt.get('specular', 1.0))
    gloss_base, gloss_range = float(opt.get('gloss_base', 0.25)), float(opt.get('gloss_range', 0.5))
    root = cfg.path(cfg.SOURCE['textures']) if cfg.SOURCE.get('textures') else None
    if root and not os.path.isdir(root):
        root = None
    work = os.path.join(cfg.OUT, 'tex')
    tex_out = os.path.join(cfg.PROJECT, 'assets', 'Textures', 'Weapons', cfg.NAME)
    mat_out = os.path.join(cfg.PROJECT, 'assets', 'Materials', 'Weapons', cfg.NAME)
    for d in (work, tex_out, mat_out):
        os.makedirs(d, exist_ok=True)
    template = os.path.join(cfg.RIG, cfg.TEMPLATE.get('material', 'TGunReceiver.bgsm'))
    missing = []
    materials = dict(json.load(open(a.parts))['materials'])
    extra = os.path.join(os.path.dirname(a.parts), 'attachments.json')
    if cfg.D.get('attachment') and os.path.exists(extra):
        for name, textures in json.load(open(extra))['materials'].items():
            # two shapes can use one diffuse texture, and only one of them names the normal map
            have = materials.get(name)
            if have is None or (len(textures) > 1 and textures[1] and not (len(have) > 1 and have[1])):
                materials[name] = textures
    for name, textures in materials.items():
        diffuse = find(root, textures[0]) if root else None
        normal = find(root, textures[1]) if root and len(textures) > 1 and textures[1] else None
        stand_in = diffuse is None

        def load(path):
            im = Image.open(path).convert('RGBA')
            if max(im.size) > max_size:
                f = max_size / max(im.size)
                im = im.resize((max(1, round(im.size[0] * f)), max(1, round(im.size[1] * f))), Image.LANCZOS)
            return im
        if stand_in:
            # no texture file of the source: one color for the material
            colors = opt.get('stand_in', {})
            color = tuple(int(v) for v in colors.get(name, colors.get('default', (70, 70, 72))))
            d = Image.new('RGBA', (512, 512), (*color, 255))
            missing.append(name)
        else:
            d = load(diffuse)
        d.convert('RGB').save(os.path.join(work, name + '_d.png'))
        if stand_in:
            Image.new('RGB', d.size, (128, 128, 255)).save(os.path.join(work, name + '_n.png'))
            mask = Image.new('L', d.size, int(opt.get('stand_in_specular', 110)))
        elif normal is not None:
            n = load(normal)
            n.convert('RGB').save(os.path.join(work, name + '_n.png'))
            mask = n.getchannel('A')
        else:
            Image.new('RGB', d.size, (128, 128, 255)).save(os.path.join(work, name + '_n.png'))
            mask = Image.new('L', d.size, 0)
        mask.point(lambda v: min(255, round(v * spec_factor))).save(os.path.join(work, name + '_spec.png'))
        mask.point(lambda v: min(255, round(255 * gloss_base + v * gloss_range))).save(os.path.join(work, name + '_gloss.png'))
        esx('texture', 'convert', os.path.join(work, name + '_d.png'), '--role', 'diffuse', '-o', os.path.join(tex_out, name + '_d.dds'), '--overwrite')
        esx('texture', 'convert', os.path.join(work, name + '_n.png'), '--role', 'normal', '-o', os.path.join(tex_out, name + '_n.dds'), '--overwrite')
        esx('texture', 'pack-channels', '--red', os.path.join(work, name + '_spec.png'), '--green', os.path.join(work, name + '_gloss.png'),
            '-o', os.path.join(tex_out, name + '_s.dds'), '--overwrite')
        bgsm = os.path.join(mat_out, name + '.bgsm')
        tmp = os.path.join(work, name + '.bgsm')
        nxt = os.path.join(work, name + '.next.bgsm')
        shutil.copyfile(template, tmp)
        for field, suffix in (('textures/diffuse', '_d'), ('textures/normal', '_n'), ('textures/smooth_specular', '_s')):
            esx('material', 'set', tmp, field, 'Weapons/%s/%s%s.dds' % (cfg.NAME, name, suffix), '-o', nxt, '--overwrite')
            os.replace(nxt, tmp)
        shutil.copyfile(tmp, bgsm)
        print('%-16s %dx%d  normal %s  specular mask %s' % (name, d.size[0], d.size[1], 'yes' if normal else 'no', mask.getextrema()))
    if missing:
        print('one color for each of these materials, because the texture files are not in the source: ' + ', '.join(missing))


if __name__ == '__main__':
    main()
