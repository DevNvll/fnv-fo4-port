#!/bin/sh
# sheet.sh fo4|src SCENE "FRAMES" VIEW [columns]  -> out/render/sheets/<kind>_<scene>_<view>.png
kind=$1; scene=$2; frames=$3; view=$4; cols=${5:-4}
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
d=out/render/tmp_${kind}_${scene}_${view}; rm -rf $d; mkdir -p $d out/render/sheets
if [ "$kind" = fo4 ]; then
  src=out/fo4/poses/$scene.npz; [ -f "$src" ] || src=out/fo4/hkx/$scene.npz
  PORT_RES=640x360 $T/bl.sh $T/blender/render_fo4.py -- --poses $src --parts $PARTS --out $d --frames "$frames" --views $view >/dev/null 2>&1
else
  PORT_RES=640x360 $T/bl.sh $T/blender/render_source.py -- --bake out/anim/$scene.json --meshes $MESHES --out $d --frames "$frames" --views $view >/dev/null 2>&1
fi
for f in $d/*.png; do n=$(basename $f | sed -E 's/.*_f([0-9]+)_.*/\1/'); magick $f -gravity NorthWest -pointsize 22 -fill yellow -annotate +6+4 "f$n" $f; done
montage $d/*.png -tile ${cols}x -geometry +2+2 out/render/sheets/${kind}_${scene}_${view}.png && rm -rf $d && echo out/render/sheets/${kind}_${scene}_${view}.png
