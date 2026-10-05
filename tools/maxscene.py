"""Read the object list of the Scene stream of a 3ds Max scene file."""
import struct, sys, os
sys.path.insert(0, os.path.dirname(__file__))
from maxchunks import parse, load_stream, read_class_directory, read_dll_directory

class Obj:
    def __init__(self, index, chunk, cls):
        self.index = index; self.chunk = chunk; self.cls = cls
        self.refs = None
        self.name = None
        self.parent = None
    @property
    def cname(self):
        return self.cls['name'] if self.cls else '?%04x' % self.chunk.id

def get_refs(chunk):
    c = chunk.find(0x2034)
    if c is not None:
        n = len(c.data) // 4
        return list(struct.unpack('<%di' % n, c.data[:n * 4]))
    c = chunk.find(0x2035)
    if c is not None:
        # first int is count?, then pairs (slot, index)
        d = c.data
        n = (len(d) - 4) // 8
        refs = {}
        for i in range(n):
            slot, idx = struct.unpack_from('<ii', d, 4 + i * 8)
            refs[slot] = idx
        if not refs:
            return []
        out = [-1] * (max(refs) + 1)
        for k, v in refs.items():
            out[k] = v
        return out
    return []

class Scene:
    def __init__(self, folder):
        self.classes = read_class_directory(os.path.join(folder, 'ClassDirectory3'))
        self.dlls = read_dll_directory(os.path.join(folder, 'DllDirectory'))
        data = load_stream(os.path.join(folder, 'Scene'))
        top = parse(data)
        assert len(top) == 1, len(top)
        self.root_id = top[0].id
        self.objs = []
        for i, c in enumerate(top[0].children):
            cls = self.classes[c.id] if c.id < len(self.classes) else None
            o = Obj(i, c, cls)
            if c.children is not None:
                o.refs = get_refs(c)
                nm = c.find(0x0962)
                if nm is not None:
                    o.name = nm.data.decode('utf-16-le')
                p = c.find(0x0960)
                if p is not None:
                    o.parent = struct.unpack_from('<i', p.data, 0)[0]
            self.objs.append(o)

    def nodes(self):
        return [o for o in self.objs if o.cls and o.cls['super'] == 0x1 and o.cls['classid'] == (1, 0)]

if __name__ == '__main__':
    s = Scene(sys.argv[1])
    print('root id %04x, objects %d' % (s.root_id, len(s.objs)))
    from collections import Counter
    cnt = Counter(o.cname for o in s.objs)
    for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]):
        print('%5d  %s' % (v, k))
