# New Vegas weapon animations for Fallout 4

These tools convert the first-person animations of a Fallout: New Vegas weapon mod to
Fallout 4 clips. One command does it, with no configuration file and no mod project:

```sh
python3 tools/port.py /path/to/extracted-mod -o /path/to/out
```

The output folder has:

* one HKX clip for each first-person clip of a Fallout 4 gun (`WPNIdleReady`,
  `WPNFireSingleReady`, `WPNReload` and so on)
* `model/NAMEReceiver.glb`: the gun of the mod with its weapon bones. The clips are for this
  gun.

The same input gives the same bytes. The command prints the animation that each clip comes
from, and the events of each reload.

| Option | Meaning |
| --- | --- |
| `-o OUT` | The output folder. Default: `NAME-fo4` in the current folder. |
| `--template NAME` | `pistol10mm` or `smg`: the vanilla weapon that gives the weapon bones, and the motion of each clip that the mod does not have. Default: `pistol10mm` when the animations have the grip code `1hp`, else `smg`. |
| `--name NAME` | The name in file names. Default: the name of the model file. |
| `--no-model` | Do not write the gun model. |
| `--esx DIR` | Write a complete esx project into DIR, to see the result in the game. |

What the clips fit:

* The hands hold the gun of the source mod, so the clips are for the model in `model/`.
* The moving parts (slide, magazine, trigger) are on the weapon bones of the template, at its
  rest places. The model has its parts on those bones.
* The clip names are those of the vanilla first-person graph of the template.

## What you need

* Python 3.11 or later with `numpy` (and `Pillow` for the textures step)
* Blender 4.5: `BLENDER` names the program (default `blender` in the `PATH`). It runs with no
  window.
* The rig folder. It has files of Fallout 4 and of the New Vegas compatibility skeleton
  (NVCS), so it is not in the repository. Make it one time from your own files:

  ```sh
  export ESX_BIN=/path/to/esx
  python3 tools/make_rig.py --data "/path/Fallout 4/Data" \
      --nvcs /path/NVCS/Meshes/Characters/_1stPerson/Skeleton.nif
  ```

  `esx` reads the game archives. The textures step and `--esx` also use it.

## In the game

Fallout 4 shows a new gun only as a weapon record with a mesh. `--esx` writes all of it:

```sh
export ESX_BIN=/path/to/esx          # esx writes the DDS and BGSM files
python3 tools/port.py /path/to/extracted-mod --esx /path/to/project
esx build /path/to/project
```

The project has the clips, the model, the textures (one color for a material with no texture
file) and three small files: `esx.toml`, an item file and a behavior source. The weapon is a
copy of the vanilla template weapon, with the name and the magazine size of the New Vegas
record. `esx probe plan` and `esx probe run` then test it in the game.

## A mod project

The project of `--esx` has a `port.toml` with three keys. Add tables to it for what the
defaults do not give (an attachment, a color, a different clip), and run it again:

```sh
python3 tools/port.py /path/to/project/port.toml
```

`examples/` has the files of three weapons: `ppk.port.toml` and `famas.port.toml` (with
attachments) and `mp7.port.toml` (a 3ds Max source).

## The configuration file

Extract the mod into the folder `source` of the esx project, beside `port.toml`. For a New
Vegas mod with a model and `.kf` animations, the file needs two keys:

```toml
name = "PPK"                    # the name in file names and paths
template = "pistol10mm"         # templates/pistol10mm.toml, or "smg"
```

The tools find each other value. `port.py CONFIG show` prints the complete data, so you can
see what they assume. A value in the file always replaces a default.

| Value | Default |
| --- | --- |
| `source` | The folder `source` beside the file. |
| `model` | The model (`.nif`) of the source with the fewest shapes, then the shortest name. |
| `animations` | The folder `_1stperson` of the source with the most `.kf` files. |
| `[source] textures` | The folder `textures` of the source. A material with no file gets one color. |
| `work` | `/tmp/NAME-port`. |
| `[weapon] trigger` | The center of the shape whose name has "trigger". |
| `[parts]` | Each node `##NAME` of the model that has a shape, by a word in its name (bar, bolt, slide, lever, clip, mag, shell, bullet, trigger, hammer). The template says which bone each kind of part gets. |
| `[glb]`, `[hkx]` | `meshes/Weapons/NAME/NAMEReceiver.glb` and `behavior/name/animations/first_person`. |
| `[fingers] min_bend` | 0.3 |
| `[collision]`, `[validate]` | The cartridges in the magazine are not colliders. |
| `[synth]` | The ready pose is the first frame of the aim clip, or of the reload. The sighted pose has the node `##SightingNode` on the view axis. |
| `[clips]` | Each animation of the source by its New Vegas name, and each other clip of a gun from the weapon motion of the template. |
| Reload events | From the motion of the magazine, the bolt and the left hand. `port.py` prints them, with `reload_seconds` for the item file. |

The New Vegas names that the tools know (the grip code at the start, for example `1hp` or
`2ha`, is not a part of the name):

| Animation | Clip |
| --- | --- |
| `aim` | `WPNIdleReady` |
| `aimis` | `WPNIdleSighted` |
| `attackright`, `attackleft`, `attack3` | `WPNFireSingleReady` |
| `attackright_a` | `WPNFireSingleReadyA` |
| `attackrightis` | `WPNFireSingleSighted` |
| `equip`, `unequip` | `WPNEquip`, `WPNUnEquip` |
| `reload...` | `WPNReload`. With two reloads, the one with `partial` in its name is `WPNReload` and the other one is `WPNReloadEmpty`. |

A sneak copy, a jam, and a variant for the empty gun or the last shot get no clip.

What a file can add:

```toml
[[attachment]]                  # one part of the gun with its own mesh
name = "Suppressor"
model = "meshes/weapons/gun/gun sil.nif"   # relative to the source folder
shapes = ["Silencer:0"]
connect = "Muzzle"              # the part has C-Muzzle, and the receiver gets P-Muzzle

[textures.stand_in]             # the color of a material with no texture file
default = [58, 58, 62]

[clips.WPNReload]               # replace the clip that the tools assume
source = "2hareloads"
events = [[21, "SoundPlay.{sound}ReloadMagOut"], [53, "reloadComplete"], [86, "reloadEnd"]]

[clips.WPNSprint]
skip = true                     # no clip of the mod: the game uses the clip of the template
```

Other tables, for what a default cannot do:

| Table | Content |
| --- | --- |
| `[source]` with `kind = "max"` | A 3ds Max source. `meshes`: the scene that gives the gun meshes. `rest`: the scene whose first frame gives the place of each gun node. It has free scene names, so the file names each part and each clip (see `examples/mp7.port.toml`). |
| `[weapon]` | `casing`: the ejection port. `muzzle_mesh`: the mesh at the muzzle (Max source). A NIF source takes the two from the nodes `ShellCasingNode` and `ProjectileNode`. |
| `[mesh]` | `static`: the meshes with no moving bone (Max source). `skip`: meshes to leave out. `[mesh.bones]`: mesh = weapon bone. |
| `[materials]` | Material name = BGSM path (Max source). A NIF source makes one material for each texture set. |
| `[[attachment]]` | `name`, `model`, `shapes`, `connect`, `at`, `projectile`. See `tools/attachments.py`. |
| `[textures]` | `max_size`, `specular`, `gloss_base`, `gloss_range`, `[textures.stand_in]`. See `tools/fnv_textures.py`. |
| `[collision]` | `skip`: shapes that are not colliders for the fingers. |
| `[fingers]` | `min_bend`: the least bend of each finger joint, as a part of its bend in the Fallout 4 reference pose. |
| `[synth]` | The clips from the weapon motion of the template. See `tools/blender/synth.py`. |
| `[clips.NAME]` | `source`, or `parts`, or `auto`, or `synth = true`, and `events`, `hold`, `skip`. See `tools/port_clips.py`. |
| `[validate]` | Options of the contact check. See `tools/blender/hand_contact.py`. |

`tools/port_defaults.py` has the rules of each default.

## The steps

`port.py CONFIG [STEP ...]` runs all steps, or the steps that you name.

| Step | Tool | Result |
| --- | --- | --- |
| `bake` | `kfbake.py`, `maxbake.py` | `WORK/out/anim/NAME.json`: the matrix of each node for each frame (30 for each second). |
| `parts` | `parts_from_nif.py`, `parts_from_max.py` | `WORK/out/fo4mesh/parts.json`: the gun meshes in Fallout 4 weapon space. |
| `attachments` | `attachments.py` | One model file for each attachment, in the project. |
| `textures` | `fnv_textures.py` | The DDS and BGSM files of each texture set, in the project. |
| `glb` | `port_glb.py` | The model file of the gun and its job file, in the project. |
| `retarget` | `blender/retarget.py` | `WORK/out/fo4/poses`: each pose on the Fallout 4 first-person skeleton. |
| `synth` | `blender/synth.py` | The clips that use the weapon motion of the template weapon. |
| `hkx` | `port_hkx.py` | `WORK/out/fo4/hkx`: the HKX clips. |
| `install` | | The clips in the project. |

`retarget` and `synth` run in Blender with no window (`tools/bl.sh`). `BLENDER` names the
Blender program. `PORT_WORK` replaces the work folder, and `PORT_RIG` replaces the rig folder.

## The two kinds of source

| `kind` | Model | Animations |
| --- | --- | --- |
| `nif` (the default) | A New Vegas model file, `.nif` (`fnvnif.py`) | New Vegas animation files, `.kf` (`fnvnif.py`, `kfbake.py`) |
| `max` | 3ds Max 2014 scenes with a CAT rig on the New Vegas skeleton (`maxscene.py`, `maxstd.py`, `maxcat.py`, `maxmesh.py`) | The same scenes |

`fnvnif.py FILE` prints the nodes and the shapes of a model, or the tracks and the text keys
of an animation. `fnvesp.py FILE.esp` prints the weapon records of a New Vegas plugin.

## What the retarget does

* The arms are solved again for the Fallout 4 bone lengths. Each wrist is at the source wrist.
* Each finger joint is on the centerline of the source finger. A finger that is too near a gun
  part moves out of it, 1.15 units for each joint at most.
* With `min_bend`, a finger that is straight in the source keeps a part of the bend of the
  Fallout 4 reference pose. The Fallout 4 hand mesh is modelled with bent fingers, and a
  fully straight finger has lumps at its joints. The finger is an arch, and its fingertip
  stays on its line.
* A weapon part follows its node of the source. A part node of a NIF source has its own place
  in the model, so the tools use the rest matrix of the node.
* A clip of `[synth]` takes the motion of the weapon bone from a vanilla clip of the template
  and applies it to a pose of the ported gun. The hands stay on the gun.
* The sighted pose of `sight = [x, y, z]` puts the eye point of the gun on the view axis.
  `sight_node` does the same with a pose of the source and a node of the model. New Vegas
  moves the camera to that node when the player aims; Fallout 4 has the sights in the clip.
* A weapon bone can have a rest rotation (`[rotations]` of the template). The mesh of the part
  is then in the turned space of the bone, so a vanilla clip shows the part at its place.
* The casing point `P-Casing` gets the rotation of the node `ShellCasingNode`. The two games
  eject a casing along the +Z axis of the point.

## Checks before a game test

```sh
export PORT_CONFIG=/path/to/project/port.toml
python3 tools/source_meshes.py                      # the mesh file of a NIF source
sh tools/validate_game.sh CLIP SCENE "0,20,40"      # source above, built clip below, from the camera of the game
sh tools/game_sheet.sh NAME "CLIP:0,5 CLIP2:3"      # built clips only
sh tools/closeup.sh SCENE FRAME NAME "VIEWS"        # close views of the hands
sh tools/bl.sh tools/blender/hand_contact.py -- --scene SCENE --frames 0,20 --kind both
```

`hand_contact.py` counts the hand skin vertices inside gun parts, for the source and for the
result. `PORT_ATTACHMENTS=1` adds the attachment meshes.

## The folders

| Folder | Content |
| --- | --- |
| `tools/` | The programs. |
| `templates/` | One file for each template weapon: `smg.toml` (the vanilla submachine gun) and `pistol10mm.toml` (the vanilla 10mm pistol). |
| `examples/` | The configuration files of three weapons. |
| `rig/` | The Fallout 4 first-person skeleton and arm meshes, the vanilla clips of each template and the New Vegas skeleton (NVCS). These are files of the game and of the NVCS mod, so only `finger_tips.json` (measurements of the two hand meshes) is in the repository. `tools/make_rig.py` makes the other files. |

## What is proved

Three weapons were converted, built with esx and tested in the game with `esx probe`:

* Shiny's HK MP7 (a 3ds Max source): 27 clips. The game test passes with 37 of 37 steps.
* FAMAS F1 (one `.kf` reload, three attachments): 26 clips. The game tests pass with 32 of 32
  and 55 of 55 steps.
* Walther PPK (9 `.kf` clips, a suppressor): 27 clips. The game tests pass with 34 of 34 and
  27 of 27 steps.

A second complete run gives the same bytes for each output file. A rig folder that
`tools/make_rig.py` makes from the game files gives the same clips.

## Limits

* Three weapons prove the tools: one Max source and two NIF sources. Each is a magazine gun.
* There are two templates, the submachine gun and the 10mm pistol. A revolver or a
  bolt-action gun needs a new template file and the clips of that vanilla weapon in `rig/`.
* The vanilla Deliverer cannot be a template now: its animation graph does not become active
  in the test game of `esx probe`, also for the vanilla record with no mod.
* A source set can have clips that the vanilla graph has no place for (a jam, an empty-gun
  pose, a last-shot clip). The tools do not connect them.
* `kfbake.py` reads transform tracks with linear, quadratic and constant keys and Euler
  rotations. It does not read B-spline data.
* `fnvnif.py` reads version 20.2.0.7 of Fallout 3 and New Vegas, with NiTriStrips and
  NiTriShape geometry. A gun with skinned parts is not supported.
* The Max reader has only the controller types of the MP7 scenes.
* The finger values were set with the MP7. The FAMAS used them, and it needed `min_bend = 0.3` for a straight thumb. The MP7 has `min_bend` off.
* The defaults are rules for the names and the motion of the mods that these three weapons
  come from. Read the output of `port.py CONFIG show` for a new mod.
* A person must look at the pictures. The checks do not judge the look.

## Origin of one file

`tools/anim_fo4.py` reads and writes the spline-compressed clips of Fallout 4. It is based on
the spline decompression of Dagobaking's skyrim-fo4-animation-conversion, which is based on
PredatorCZ/HavokLib (GPL-3.0).
