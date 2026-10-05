"""Print the blend hint of the animation binding of each HKX clip (0 normal, 1 additive)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import anim_fo4 as a

for path in sys.argv[1:]:
    data = open(path, 'rb').read()
    sections = a._parse_hkx_sections(data)
    cn = sections['__classnames__']; ds = sections['__data__']
    objects = a._parse_virtual_fixups(data, ds, cn['offset'])
    out = []
    for rel, cls in objects:
        if cls == 'hkaAnimationBinding':
            out.append(data[ds['offset'] + rel + 0x50])
    kinds = [c for _, c in objects if c.startswith('hka') and 'Animation' in c and c not in ('hkaAnimationContainer', 'hkaAnimationBinding')]
    print('%-44s blend hint %s  %s' % (os.path.basename(path), out, kinds[0] if kinds else ''))
