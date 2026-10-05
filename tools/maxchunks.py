"""Minimal reader of the chunk trees in the streams of a 3ds Max scene file."""
import struct, zlib, sys, os

def load_stream(path):
    data = open(path, 'rb').read()
    if data[:2] == b'\x1f\x8b':
        data = zlib.decompress(data, 16 + zlib.MAX_WBITS)
    return data

class Chunk:
    __slots__ = ('id', 'data', 'children', 'offset')
    def __init__(self, cid, data, children, offset):
        self.id = cid; self.data = data; self.children = children; self.offset = offset
    def is_container(self):
        return self.children is not None
    def find(self, cid):
        if self.children:
            for c in self.children:
                if c.id == cid:
                    return c
        return None
    def find_all(self, cid):
        return [c for c in (self.children or []) if c.id == cid]

def parse(data, start=0, end=None, depth=0):
    out = []
    pos = start
    if end is None:
        end = len(data)
    while pos < end:
        if pos + 6 > end:
            break
        cid, size = struct.unpack_from('<HI', data, pos)
        hdr = 6
        if size == 0:
            size, = struct.unpack_from('<Q', data, pos + 6)
            hdr = 14
            container = bool(size & (1 << 63))
            size &= ~(1 << 63)
        else:
            container = bool(size & (1 << 31))
            size &= ~(1 << 31)
        if size < hdr or pos + size > end:
            raise ValueError('bad chunk at %d id %04x size %d end %d' % (pos, cid, size, end))
        body_start = pos + hdr
        body_end = pos + size
        if container:
            children = parse(data, body_start, body_end, depth + 1)
            out.append(Chunk(cid, None, children, pos))
        else:
            out.append(Chunk(cid, data[body_start:body_end], None, pos))
        pos = body_end
    return out

def read_class_directory(path):
    chunks = parse(load_stream(path))
    classes = []
    for c in chunks:
        # 0x2040 container: 0x2060 header (dll index, classid a,b, superclass), 0x2042 name
        hdr = c.find(0x2060)
        name = c.find(0x2042)
        dll, a, b, sid = struct.unpack('<iIII', hdr.data[:16])
        classes.append({'dll': dll, 'classid': (a, b), 'super': sid,
                        'name': name.data.decode('utf-16-le') if name else ''})
    return classes

def read_dll_directory(path):
    chunks = parse(load_stream(path))
    dlls = []
    for c in chunks:
        if c.children is None:
            continue
        d = c.find(0x2039); n = c.find(0x2037)
        dlls.append({'desc': d.data.decode('utf-16-le') if d else '', 'file': n.data.decode('utf-16-le') if n else ''})
    return dlls

if __name__ == '__main__':
    folder = sys.argv[1]
    classes = read_class_directory(os.path.join(folder, 'ClassDirectory3'))
    dlls = read_dll_directory(os.path.join(folder, 'DllDirectory'))
    for i, c in enumerate(classes):
        dll = dlls[c['dll']]['file'] if 0 <= c['dll'] < len(dlls) else 'builtin(%d)' % c['dll']
        print('%3d  %08x:%08x  super %08x  %-40s %s' % (i, c['classid'][0], c['classid'][1], c['super'], c['name'], dll))
