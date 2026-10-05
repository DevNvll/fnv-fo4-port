"""Convert the first-person animations of a New Vegas weapon mod to Fallout 4.

    python3 tools/port.py SOURCE [-o OUT] [--template NAME] [--name NAME] [--no-model] [--esx DIR]

SOURCE is the folder of the extracted mod. The result is one HKX clip for each first-person
clip of a Fallout 4 gun, in the folder OUT (default: NAME-fo4 in the current folder). No
configuration file and no mod project are necessary. The tools find the model, the
animations, the gun parts, the clips and their events (tools/port_defaults.py).

    --template  the vanilla weapon that gives the weapon bones and the motion of the clips
                that the mod does not have: pistol10mm or smg. Default: pistol10mm when the
                animations have the grip code 1hp, else smg.
    --name      the name in file names. Default: the name of the model file.
    --no-model  do not write the gun model. By default OUT/model/NAMEReceiver.glb has the gun
                of the mod with its weapon bones: the clips are for that gun.
    --esx DIR   write a complete esx project into DIR instead: the clips, the model, the
                textures and the three small files that make a weapon of the gun (the values
                of the template weapon). `esx build DIR` then makes the plugin.

A project with a configuration file can do more (attachments, textures, the files of an esx
mod project):

    python3 tools/port.py PROJECT/port.toml [STEP ...]

The steps, in this order (all steps when none is given):

    bake      evaluate each source animation and write the node matrices (out/anim/NAME.json)
    parts     put the gun meshes into Fallout 4 weapon space (out/fo4mesh/parts.json)
    attachments  write the model of each attachment of a NIF source into the project
    textures  convert the textures of a NIF source (project assets/Textures and assets/Materials)
    glb       write the model file and its job file into the project
    retarget  move each pose to the Fallout 4 first-person skeleton (out/fo4/poses)
    synth     make the clips that use the weapon motion of the template weapon
    hkx       write the HKX clips (out/fo4/hkx)
    install   copy the clips into the project

`port.py CONFIG show` prints the complete data: the values of the file and each default.
`port.py CONFIG plan` prints the source of each clip and the events of each reload; the bake
step prints the same.

The same input gives the same bytes. The work folder is `work` of the configuration file
(default /tmp/NAME-port).
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STEPS = ['bake', 'parts', 'attachments', 'textures', 'glb', 'retarget', 'synth', 'hkx', 'install']


def from_folder(argv):
    """The animations of a source folder, with no configuration file: write a small file into
    the work folder and return its name and the steps."""
    import argparse
    import json
    import re
    sys.path.insert(0, HERE)
    import port_defaults
    ap = argparse.ArgumentParser(prog='port.py', description='Convert the first-person animations of a New Vegas weapon mod to Fallout 4.')
    ap.add_argument('source', help='the folder of the extracted mod')
    ap.add_argument('-o', '--output', help='the folder for the HKX clips (default: NAME-fo4 in the current folder)')
    ap.add_argument('--template', help='pistol10mm or smg (default: by the grip code of the animations)')
    ap.add_argument('--name', help='the name in file names (default: the name of the model file)')
    ap.add_argument('--no-model', action='store_true', help='do not write the gun model')
    ap.add_argument('--esx', metavar='DIR', help='write a complete esx project into DIR')
    a = ap.parse_args(argv)
    source = os.path.abspath(a.source)
    name = a.name or re.sub(r'[^A-Za-z0-9]', '', os.path.splitext(os.path.basename(port_defaults.find_model(source)))[0]).upper()
    template = a.template
    if template is None:
        folder = port_defaults.find_animations(source)
        if folder is None:
            raise SystemExit('the folder %s has no _1stperson folder with .kf files' % source)
        template = 'pistol10mm' if any(f.lower().startswith('1hp') for f in os.listdir(folder)) else 'smg'
    work = '/tmp/%s-port' % name.lower()
    os.makedirs(work, exist_ok=True)
    lines = ['# written by tools/port.py for the folder %s' % source]
    lines += ['%s = %s' % (key, json.dumps(value)) for key, value in (('name', name), ('template', template), ('source', source))]
    if a.esx:
        # a project: each output goes to its place of an esx project, by the defaults
        out = os.path.abspath(a.esx)
        os.makedirs(out, exist_ok=True)
        config = os.path.join(out, 'port.toml')
        steps = STEPS + ['esxfiles']
        print('[port] %s: template %s, esx project in %s' % (name, template, out), flush=True)
    else:
        out = os.path.abspath(a.output or name + '-fo4')
        config = os.path.join(work, 'port.toml')
        lines += ['', '[hkx]', 'output = %s' % json.dumps(out), '', '[glb]',
                  'output = %s' % json.dumps(os.path.join(out, 'model', name + 'Receiver.glb'))]
        steps = ['bake', 'parts'] + ([] if a.no_model else ['glb']) + ['retarget', 'synth', 'hkx', 'install']
        print('[port] %s: template %s, clips to %s' % (name, template, out), flush=True)
    with open(config, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return config, steps


def esx_files(cfg):
    """The three small files that make a weapon of the gun in an esx project: the manifest,
    the item file and the behavior source. A file that is there stays as it is."""
    sys.path.insert(0, HERE)
    import fnvesp
    import struct
    name, t = cfg.NAME, cfg.TEMPLATE
    project = cfg.PROJECT
    title, capacity = name, None
    # the name and the magazine size of the weapon record of the New Vegas plugin
    folder = cfg.SOURCE.get('folder', '')
    model = os.path.relpath(cfg.SOURCE['model'], folder).replace('/', '\\').lower()
    model = model[len('meshes\\'):] if model.startswith('meshes\\') else model
    for f in sorted(os.listdir(folder)) if os.path.isdir(folder) else []:
        if f.lower().endswith(('.esp', '.esm')):
            d = open(os.path.join(folder, f), 'rb').read()
            for sig, _, body in fnvesp.records(d, 0, len(d)):
                if sig != 'WEAP':
                    continue
                rec = {}
                for fs, v in fnvesp.fields(body):
                    if fs in ('FULL', 'MODL'):
                        rec.setdefault(fs, fnvesp.text(v))
                    elif fs == 'DATA' and len(v) >= 15:
                        rec['clip'] = struct.unpack_from('<iifhB', v, 0)[4]
                if rec.get('MODL', '').lower() == model:
                    title, capacity = rec.get('FULL', name), rec.get('clip')
    end = None
    reload_clip = cfg.D.get('clips', {}).get('WPNReload', {})
    for frame, text in reload_clip.get('events', []):
        if text == 'reloadEnd':
            end = frame / 30.0
    written = []

    def write(rel, text):
        p = os.path.join(project, rel)
        if os.path.exists(p):
            return
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, 'w') as f:
            f.write(text)
        written.append(rel)
    write('esx.toml', '# %s for Fallout 4: the gun and the first-person animations of a New Vegas mod.\n'
          'plugin = "%s.esp"\nauthor = "Port made with esx"\n'
          'description = "%s with the first-person animations of the New Vegas mod, on the Fallout 4 rig."\n\n[meshes]\nscale = 1.0\n' % (title, name, title))
    item = ['# The weapon. `esx build` makes the records from this file. `esx help-topic items` gives the keys.',
            '# The weapon is a copy of the vanilla record %s: it has its damage, its ammunition, its sounds' % t['record'],
            '# and its third-person animations. Give a key here to change a value.',
            'kind = "weapon"', 'name = %s' % json_string(title), 'prefix = "%s"' % name, 'from = "%s"' % t['record']]
    if capacity:
        item.append('capacity = %d' % capacity)
    if end:
        item.append('reload_seconds = %.4f' % end)
    item += ["model = '%s'" % t['dummy_model'], '',
             '# One part has the whole gun. Its mesh has the weapon bones that the animations move.',
             '[part.Receiver]', 'from = "%s"' % t['receiver'], 'name = %s' % json_string(title + ' Receiver'),
             "model = 'Weapons\\%s\\%sReceiver.nif'" % (name, name), '']
    write('items/10-%s.toml' % name.lower(), '\n'.join(item))
    bhv = ['mod %s {' % name, '  plugin "%s.esp"' % name, '  prefix %s' % name, '}', '', 'weapon %s like %s {' % (name, t['anims'])]
    if 'WPNReloadEmpty' in cfg.D.get('clips', {}):
        bhv += ['  reload {', '    views first_person, power_armor_first_person', '    when empty play WPNReloadEmpty', '  }']
    bhv += ['}', '']
    write('behavior/%s/%s.bhv' % (name.lower(), name.lower()), '\n'.join(bhv))
    print('[port] esx project files: %s' % (', '.join(written) if written else 'none written, each file is there'))
    print('[port] build the plugin with: esx build %s' % project)


def json_string(text):
    import json
    return json.dumps(text)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    if os.path.isdir(sys.argv[1]):
        config, default_steps = from_folder(sys.argv[1:])
        given = []
    else:
        config, default_steps, given = sys.argv[1], STEPS, sys.argv[2:]
    os.environ['PORT_CONFIG'] = os.path.abspath(config)
    sys.path.insert(0, HERE)
    import port_config as cfg
    import port_clips
    os.environ['PORT_WORK'] = cfg.WORK
    steps = given or default_steps
    if steps == ['show']:
        # the complete data: the file and each default
        import port_defaults
        print(port_defaults.dump({k: v for k, v in cfg.D.items() if k != 'assumed_clips'}), end='')
        return
    for s in steps:
        if s not in STEPS and s not in ('plan', 'esxfiles'):
            raise SystemExit('unknown step %s; the steps are: %s, and show, plan, esxfiles' % (s, ' '.join(STEPS)))
    src = cfg.D['source']
    anim = os.path.join(cfg.OUT, 'anim')
    parts = os.path.join(cfg.OUT, 'fo4mesh', 'parts.json')
    fo4 = os.path.join(cfg.OUT, 'fo4')
    hkx = os.path.join(fo4, 'hkx')
    clips = port_clips.clips()
    # the pose files that the clips and the synth step use
    scenes = []
    for c in clips.values():
        for scene, _, _ in c['parts']:
            if not scene.startswith('synth_') and scene not in scenes:
                scenes.append(scene)
    for kind in ('ready', 'sighted'):
        b = cfg.D.get('synth', {}).get(kind, {}).get('bake')
        if b and b not in scenes:
            scenes.append(b)
    # the synth step reads these bake files itself; a Max source must have them too
    synth_bakes = [st['bake'] for st in cfg.D.get('synth', {}).get('static', {}).values() if 'bake' in st]
    if steps == ['plan']:
        # what the tools assume for the clips: the source of each clip and the events of each reload
        for name, source in cfg.D.get('assumed_clips', {}).items():
            print('[port] %-22s <- %s' % (name, source))
        for name, c in cfg.D.get('clips', {}).items():
            if name.startswith('WPNReload') and c.get('events'):
                end = [f for f, t in c['events'] if t == 'reloadEnd']
                print('[port] %s events: %s' % (name, ', '.join('%d %s' % (f, t.split('.')[-1].replace('{sound}', '')) for f, t in c['events'])))
                if end and name == 'WPNReload':
                    print('[port] reload_seconds for the item file: %.4f' % (end[0] / 30.0))
        return

    def run(args, label):
        print('[port] ' + label, flush=True)
        r = subprocess.run(args)
        if r.returncode != 0:
            raise SystemExit('[port] the step failed: ' + label)

    def py(script, *args):
        return [sys.executable, os.path.join(HERE, script), *args]

    def blender(script, *args):
        return ['sh', os.path.join(HERE, 'bl.sh'), os.path.join(HERE, 'blender', script), '--', *args]

    for step in steps:
        if step == 'bake':
            os.makedirs(anim, exist_ok=True)
            if src['kind'] == 'nif':
                for name, kf in src.get('animations', {}).items():
                    run(py('kfbake.py', cfg.path(kf), '--model', cfg.path(src['model']), '-o', os.path.join(anim, name + '.json')), 'bake ' + name)
            else:
                for name in scenes + synth_bakes:
                    out = os.path.join(anim, name + '.json')
                    if not os.path.exists(out):
                        run(py('maxbake.py', os.path.join(cfg.WORK, 'ole', name), '-o', out), 'bake ' + name)
            subprocess.run([sys.executable, os.path.abspath(__file__), os.environ['PORT_CONFIG'], 'plan'])
        elif step == 'parts':
            if src['kind'] == 'nif':
                run(py('parts_from_nif.py', '-o', parts), 'parts')
            else:
                run(py('parts_from_max.py', '--meshes', os.path.join(cfg.OUT, 'mesh', src['meshes'], 'meshes.json'),
                       '--rest', os.path.join(anim, src['rest'] + '.json'), '-o', parts), 'parts')
        elif step == 'attachments':
            if src['kind'] == 'nif' and cfg.D.get('attachment'):
                run(py('attachments.py'), 'attachments')
        elif step == 'textures':
            if src['kind'] == 'nif':
                run(py('fnv_textures.py'), 'textures')
        elif step == 'glb':
            run(py('port_glb.py', '--parts', parts), 'glb')
        elif step == 'retarget':
            args = []
            for name in scenes:
                args += ['--bake', os.path.join(anim, name + '.json')]
            run(blender('retarget.py', *args, '--out', fo4), 'retarget %d pose files' % len(scenes))
        elif step == 'synth':
            if cfg.D.get('synth'):
                run(blender('synth.py', '--out', fo4), 'synth')
        elif step == 'hkx':
            run(py('port_hkx.py', '--poses', os.path.join(fo4, 'poses'), '--out', hkx), 'hkx')
        elif step == 'esxfiles':
            # in a new process: the reload time needs the bake files
            if os.environ.get('PORT_ESXFILES') == '1':
                esx_files(cfg)
            else:
                subprocess.run([sys.executable, os.path.abspath(__file__), os.environ['PORT_CONFIG'], 'esxfiles'],
                               env=dict(os.environ, PORT_ESXFILES='1'))
        elif step == 'install':
            out = cfg.path(cfg.D['hkx']['output'])
            os.makedirs(out, exist_ok=True)
            for name in clips:
                shutil.copyfile(os.path.join(hkx, name + '.hkx'), os.path.join(out, name + '.hkx'))
            print('[port] %d clips in %s' % (len(clips), out))


if __name__ == '__main__':
    main()
