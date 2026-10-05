import sys, os, struct
sys.path.insert(0, os.path.dirname(__file__))
from maxscene import Scene

def hexs(b, n=48):
    return b[:n].hex() + ('..' if len(b) > n else '')

def dump_chunks(chunks, indent, maxdepth=6, skip=()):
    for c in chunks:
        if c.id in skip:
            continue
        if c.children is not None:
            print('%s[%04x] container (%d)' % (indent, c.id, len(c.children)))
            if maxdepth > 0:
                dump_chunks(c.children, indent + '  ', maxdepth - 1, skip)
        else:
            print('%s[%04x] %d: %s' % (indent, c.id, len(c.data), hexs(c.data)))

def dump_obj(s, idx, depth=0, maxdepth=4, seen=None, chunks=True):
    if seen is None:
        seen = set()
    ind = '  ' * depth
    if idx < 0 or idx >= len(s.objs):
        print('%s<%d>' % (ind, idx)); return
    o = s.objs[idx]
    print('%s#%d %s %s refs=%s' % (ind, idx, o.cname, repr(o.name) if o.name else '', o.refs))
    if idx in seen:
        print('%s  (seen)' % ind); return
    seen.add(idx)
    if chunks and o.chunk.children is not None:
        dump_chunks(o.chunk.children, ind + '   | ', 3, skip=(0x2034, 0x2035))
    if depth < maxdepth and o.refs:
        for r in o.refs:
            if r >= 0:
                dump_obj(s, r, depth + 1, maxdepth, seen, chunks)

if __name__ == '__main__':
    s = Scene(sys.argv[1])
    idx = int(sys.argv[2])
    md = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    dump_obj(s, idx, 0, md)
