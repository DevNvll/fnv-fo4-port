#!/bin/sh
# closeup.sh SCENE FRAME NAME [VIEWS]: close views of the hands, rows: source, then each pose folder in $SETS
T=$(cd "$(dirname "$0")" && pwd)
: "${PORT_CONFIG:?Set PORT_CONFIG to the port.toml of the weapon}"
eval "$(python3 "$T/port_env.py")" || exit 1
cd "$PORT_WORK" || exit 1
S=$1; F=$2; N=$3
V=${4:-"o200_10_26_50_0_9_-3,o160_-35_26_50_0_9_-3,o20_5_24_50_0_-2_-3,o-20_-40_24_50_0_-2_-3,o90_-20_26_50_0_4_-3,o270_-60_26_50_0_4_-3"}
SETS=${SETS:-"fo4_rot fo4_path fo4"}
D=out/render/closeup_$N
rm -rf $D; mkdir -p $D
PORT_RES=640x480 $T/bl.sh $T/blender/render_source.py -- --bake out/anim/$S.json --meshes $MESHES --out $D/0src --frames $F --views "$V" 2>&1 | grep -E "Error|Trace"
i=1
for set in $SETS; do
  PORT_RES=640x480 $T/bl.sh $T/blender/render_fo4.py -- --poses out/$set/poses/$S.npz --parts $PARTS --out $D/$i$set --frames $F --views "$V" 2>&1 | grep -E "Error|Trace"
  i=$((i+1))
done
rows=$(/bin/ls -d $D/*/ | wc -l)
cols=$(echo "$V" | tr ',' '\n' | wc -l)
montage $(for d in $(/bin/ls -d $D/*/ | sort); do /bin/ls $d*.png | sort; done) -tile ${cols}x${rows} -geometry ${TILE:-420}x+2+2 out/render/closeup_$N.png && echo out/render/closeup_$N.png
