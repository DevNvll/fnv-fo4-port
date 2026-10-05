"""Give the files of an extracted MSI cabinet their real paths (File, Component, Directory tables)."""
import struct, sys, os, shutil
src, cab, out = sys.argv[1], sys.argv[2], sys.argv[3]
pool = open(os.path.join(src, '!_StringPool'), 'rb').read()
data = open(os.path.join(src, '!_StringData'), 'rb').read()
strings = ['']
pos = 0
i = 4
wide = bool(struct.unpack_from('<H', pool, 2)[0] & 0x8000)
while i < len(pool):
    ln, rc = struct.unpack_from('<HH', pool, i); i += 4
    if ln == 0 and rc != 0:
        ln2, rc2 = struct.unpack_from('<HH', pool, i); i += 4
        ln = (rc << 16) | ln2
    strings.append(data[pos:pos + ln].decode('latin-1')); pos += ln
ssz = 3 if (len(strings) > 65536 or wide) else 2
def read_table(name, cols):
    raw = open(os.path.join(src, '!' + name), 'rb').read()
    sizes = [ssz if c == 's' else (2 if c == 'i2' else 4) for c in cols]
    n = len(raw) // sum(sizes)
    assert n * sum(sizes) == len(raw), (name, len(raw), sum(sizes))
    rows = [[] for _ in range(n)]
    off = 0
    for c, sz in zip(cols, sizes):
        for r in range(n):
            b = raw[off + r * sz: off + r * sz + sz]
            v = int.from_bytes(b, 'little')
            if c == 's':
                rows[r].append(strings[v] if v < len(strings) else '?')
            elif c == 'i2':
                rows[r].append(v - 0x8000)
            else:
                rows[r].append(v - 0x80000000)
        off += n * sz
    return rows
files = None
for seq in ('i4', 'i2'):
    try:
        files = read_table('File', ['s', 's', 's', 'i4', 's', 's', 'i2', seq]); break
    except AssertionError as e:
        err = e
if files is None:
    raise err
comps = {r[0]: r[2] for r in read_table('Component', ['s', 's', 's', 'i2', 's', 's'])}
dirs = {r[0]: (r[1], r[2]) for r in read_table('Directory', ['s', 's', 's'])}
def dirpath(d, depth=0):
    if d not in dirs or depth > 40:
        return ''
    parent, name = dirs[d]
    name = name.split('|')[-1].split(':')[0]
    if name == '.':
        name = ''
    if not parent or parent == d:
        return name
    p = dirpath(parent, depth + 1)
    return os.path.join(p, name) if name else p
n = 0
for key, comp, fname, size, ver, lang, attr, seq in files:
    name = fname.split('|')[-1]
    d = dirpath(comps.get(comp, ''))
    srcf = os.path.join(cab, key)
    if not os.path.exists(srcf):
        continue
    dst = os.path.join(out, d, name)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(srcf, dst); n += 1
print('files', n)
