#!/bin/sh
# game_sheet.sh NAME "CLIP:FRAMES CLIP:FRAMES ..." [COLUMNS]
# Decode built HKX clips of the project and render them from the camera of the game (16:9, a
# view angle of 87 degrees). One picture for each frame, in one sheet: out/validate/game/NAME.png
name=$1; list=$2; cols=${3:-4}
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
d=out/validate/s_$name; rm -rf $d; mkdir -p $d out/validate/game
files=""
for item in $list; do
  clip=${item%%:*}; frames=${item#*:}
  python3 $T/hkx_to_npz.py $CLIPS/$clip.hkx -o $d/$clip.npz > /dev/null || exit 1
  PORT_RES=${RES:-1280x720} $T/bl.sh $T/blender/render_fo4.py -- --poses $d/$clip.npz --parts $PARTS --out $d/$clip --frames "$frames" --views fp --fov 87 --tag x >/dev/null 2>&1 || exit 1
  for f in $(echo "$frames" | tr ',' ' '); do ff=$(printf %03d $f); magick $d/$clip/x_f${ff}_fp.png -gravity NorthWest -pointsize 30 -fill cyan -annotate +8+4 "$clip f$f" $d/$clip/a_$ff.png; files="$files $d/$clip/a_$ff.png"; done
done
montage $files -tile ${cols}x -geometry ${TILE:-640}x+2+2 out/validate/game/$name.png && echo out/validate/game/$name.png
