"""Print the weapon records of a Fallout: New Vegas plugin.

    python3 tools/fnvesp.py FILE.esp

For each WEAP record: the editor ID, the name, the model and the main values of DATA and DNAM.
The values are a guide for the item file of the Fallout 4 mod. The two games have different scales.
"""
import struct
import sys
import zlib


def records(d, pos, end):
    while pos < end:
        sig = d[pos:pos + 4]
        size, = struct.unpack_from('<I', d, pos + 4)
        if sig == b'GRUP':
            yield from records(d, pos + 24, pos + size)
            pos += size
        else:
            flags, = struct.unpack_from('<I', d, pos + 8)
            body = d[pos + 24:pos + 24 + size]
            if flags & 0x00040000:
                body = zlib.decompress(body[4:])
            yield sig.decode('latin1'), struct.unpack_from('<I', d, pos + 12)[0], body
            pos += 24 + size


def fields(body):
    p = 0
    while p < len(body):
        sig = body[p:p + 4].decode('latin1')
        n, = struct.unpack_from('<H', body, p + 4)
        yield sig, body[p + 6:p + 6 + n]
        p += 6 + n


def text(b):
    return b.rstrip(b'\0').decode('latin1')


def main(path):
    d = open(path, 'rb').read()
    counts = {}
    for sig, form, body in records(d, 0, len(d)):
        counts[sig] = counts.get(sig, 0) + 1
        if sig != 'WEAP':
            continue
        out = {'form': '%08X' % form}
        for f, v in fields(body):
            if f in ('EDID', 'FULL', 'MODL', 'ICON'):
                out.setdefault(f, text(v))
            elif f == 'DATA' and len(v) >= 15:
                out['value'], out['health'], out['weight'], out['damage'], out['clip'] = struct.unpack_from('<iifhB', v, 0)
            elif f == 'DNAM' and len(v) >= 40:
                out['animation_type'], out['animation_multiplier'], out['reach'] = struct.unpack_from('<Iff', v, 0)
                out['min_spread'], out['spread'] = struct.unpack_from('<ff', v, 20)
                out['projectiles'] = v[37] if len(v) > 37 else None
                if len(v) >= 68:
                    out['fire_rate'] = round(struct.unpack_from('<f', v, 64)[0], 3)
            elif f in ('ENAM', 'NAM0'):
                out.setdefault('ammo_form' if f == 'ENAM' else 'ammo_list', '%08X' % struct.unpack_from('<I', v, 0)[0])
        print(out)
    print('records', counts)


if __name__ == '__main__':
    main(sys.argv[1])
