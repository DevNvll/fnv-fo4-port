#!/bin/sh
# fpcmp.sh SCENE "FRAMES" NAME: the first-person view of the source and of each pose set in $SETS, one column for each frame
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
S=$1; F=$2; N=$3
SETS=${SETS:-"fo4_rs fo4"}
VIEW=${VIEW:-fp}
D=out/render/fpcmp_$N
rm -rf $D; mkdir -p $D
PORT_RES=${RES:-1280x960} $T/bl.sh $T/blender/render_source.py -- --bake out/anim/$S.json --meshes $MESHES --out $D/0src --frames $F --views "$VIEW" 2>&1 | grep -E "Error|Trace"
i=1
for set in $SETS; do
  PORT_RES=${RES:-1280x960} $T/bl.sh $T/blender/render_fo4.py -- --poses out/$set/poses/$S.npz --parts $PARTS --out $D/$i$set --frames $F --views "$VIEW" 2>&1 | grep -E "Error|Trace"
  i=$((i+1))
done
cols=$(echo "$F" | tr ',' '\n' | wc -l)
montage $(for d in $(/bin/ls -d $D/*/ | sort); do /bin/ls $d*.png | sort; done) -tile ${cols}x -geometry ${TILE:-640}x+2+2 out/render/fpcmp_$N.png && echo out/render/fpcmp_$N.png
