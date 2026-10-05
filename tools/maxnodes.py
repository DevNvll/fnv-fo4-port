import sys, os, struct
sys.path.insert(0, os.path.dirname(__file__))
from maxscene import Scene

s = Scene(sys.argv[1])
nodes = s.nodes()
byidx = {o.index: o for o in s.objs}
def ctl_desc(idx, depth=0):
    if idx is None or idx < 0 or idx not in byidx:
        return '-'
    o = byidx[idx]
    return o.cname
for n in nodes:
    refs = n.refs or []
    tm = ctl_desc(refs[0]) if len(refs) > 0 else '-'
    ob = ctl_desc(refs[1]) if len(refs) > 1 else '-'
    par = byidx[n.parent].name if n.parent in byidx and byidx[n.parent].name else ('ROOT' if n.parent in byidx and byidx[n.parent].cname == 'RootNode' else str(n.parent))
    print('%5d %-34s parent=%-28s tm=%-24s obj=%s' % (n.index, n.name, par, tm, ob))
