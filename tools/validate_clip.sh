#!/bin/sh
# validate_clip.sh CLIP SCENE "FRAMES"
# Decode the built HKX clip, render it (FO4 arms and the gun) and the source scene at the
# same frames, and write two comparison sheets: first-person view and side view.
clip=$1; scene=$2; frames=$3
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
d=out/validate/$clip; rm -rf $d; mkdir -p $d out/validate/sheets
python3 $T/hkx_to_npz.py $CLIPS/$clip.hkx -o $d/hkx.npz > $d/decode.txt || exit 1
PORT_RES=480x270 $T/bl.sh $T/blender/render_fo4.py -- --poses $d/hkx.npz --parts $PARTS --out $d/fo4 --frames "$frames" --views fp,side --tag x >/dev/null 2>&1 || exit 1
if [ -n "$scene" ]; then
  PORT_RES=480x270 $T/bl.sh $T/blender/render_source.py -- --bake out/anim/$scene.json --meshes $MESHES --out $d/src --frames "$frames" --views fp,side >/dev/null 2>&1 || exit 1
fi
n=$(echo "$frames" | tr ',' '\n' | wc -l)
for v in fp side; do
  list=""
  if [ -n "$scene" ]; then for f in $(echo "$frames" | tr ',' ' '); do ff=$(printf %03d $f); magick $d/src/${scene}_f${ff}_$v.png -gravity NorthWest -pointsize 18 -fill yellow -annotate +5+3 "source f$f" $d/src/a_${ff}_$v.png; list="$list $d/src/a_${ff}_$v.png"; done; fi
  for f in $(echo "$frames" | tr ',' ' '); do ff=$(printf %03d $f); magick $d/fo4/x_f${ff}_$v.png -gravity NorthWest -pointsize 18 -fill cyan -annotate +5+3 "FO4 HKX f$f" $d/fo4/a_${ff}_$v.png; list="$list $d/fo4/a_${ff}_$v.png"; done
  montage $list -tile ${n}x -geometry +2+2 out/validate/sheets/${clip}_$v.png
done
cat $d/decode.txt
rm -rf $d/src $d/fo4
