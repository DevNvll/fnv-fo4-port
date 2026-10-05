#!/bin/sh
# validate_game.sh CLIP SCENE "FRAMES" [CROP]
# Decode the HKX clip of the project and render it from the camera of the game (16:9, the
# fitted view angle of 87 degrees), with the source scene at the same frames above it.
clip=$1; scene=$2; frames=$3; crop=${4:-800x450+440+270}; out=${5:-$1}
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
d=out/validate/g_$out; rm -rf $d; mkdir -p $d out/validate/game
python3 $T/hkx_to_npz.py $CLIPS/$clip.hkx -o $d/hkx.npz > $d/decode.txt || exit 1
PORT_RES=1280x720 $T/bl.sh $T/blender/render_fo4.py -- --poses $d/hkx.npz --parts $PARTS --out $d/fo4 --frames "$frames" --views fp --fov 87 --tag x >/dev/null 2>&1 || exit 1
PORT_RES=1280x720 $T/bl.sh $T/blender/render_source.py -- --bake out/anim/$scene.json --meshes $MESHES --out $d/src --frames "$frames" --views fp --fov 87 >/dev/null 2>&1 || exit 1
n=$(echo "$frames" | tr ',' '\n' | wc -l)
list=""
for f in $(echo "$frames" | tr ',' ' '); do ff=$(printf %03d $f); magick $d/src/${scene}_f${ff}_fp.png -crop $crop +repage -gravity NorthWest -pointsize 22 -fill yellow -annotate +6+4 "source f$f" $d/src/a_$ff.png; list="$list $d/src/a_$ff.png"; done
for f in $(echo "$frames" | tr ',' ' '); do ff=$(printf %03d $f); magick $d/fo4/x_f${ff}_fp.png -crop $crop +repage -gravity NorthWest -pointsize 22 -fill cyan -annotate +6+4 "FO4 clip f$f" $d/fo4/a_$ff.png; list="$list $d/fo4/a_$ff.png"; done
montage $list -tile ${n}x -geometry ${TILE:-480}x+2+2 out/validate/game/$out.png && echo out/validate/game/$out.png
rm -rf $d/src $d/fo4
