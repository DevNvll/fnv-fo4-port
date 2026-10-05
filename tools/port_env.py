"""Print shell variables for the helper scripts: the folders and files of the weapon in PORT_CONFIG."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_config as cfg
import source_meshes

print('export PORT_WORK="%s"' % cfg.WORK)
print('CLIPS="%s"' % cfg.path(cfg.D['hkx']['output']))
print('MESHES="%s"' % source_meshes.path())
print('PARTS="%s"' % os.path.join(cfg.OUT, 'fo4mesh', 'parts.json'))
