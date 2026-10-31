#!/usr/bin/env bash
# 双臂关节拖拽：默认 10s，Ctrl+C 提前结束；每臂进程内会 disableDrag + 下电。
# 用法：
#   bash scripts/dual_arm_drag.sh
#   bash scripts/dual_arm_drag.sh 10
#   DURATION_S=10 bash scripts/dual_arm_drag.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/examples/ar3_right_arm_status/build/drag_joint_torque_100ms"
LOCAL="${LOCAL_IP:-192.168.2.100}"
LEFT="${LEFT_IP:-192.168.2.160}"
RIGHT="${RIGHT_IP:-192.168.2.161}"
DUR="${1:-${DURATION_S:-10}}"
OUT="${OUT_DIR:-/tmp}"
STAMP=$(date +%Y%m%d_%H%M%S)

if [[ ! -x "$BIN" ]]; then
  echo "missing $BIN — build first:" >&2
  echo "  cmake -S $ROOT/examples/ar3_right_arm_status -B $ROOT/examples/ar3_right_arm_status/build -DXCORE_SDK_ROOT=$ROOT/third_party/xCoreSDK-CPP" >&2
  echo "  cmake --build $ROOT/examples/ar3_right_arm_status/build --target drag_joint_torque_100ms" >&2
  exit 1
fi

# 避免 root 占用日志导致无权写
chmod -R u+rwX "$ROOT/examples/ar3_right_arm_status/logs" 2>/dev/null || true

echo "dual drag ${DUR}s  L=$LEFT  R=$RIGHT  (Ctrl+C to stop → each arm powers off)"
cd "$ROOT/examples/ar3_right_arm_status"

pids=()
cleanup() {
  echo
  echo "stopping children..."
  for p in "${pids[@]:-}"; do
    kill -INT "$p" 2>/dev/null || true
  done
  wait || true
}
trap cleanup INT TERM

"$BIN" "$LEFT" "$LOCAL" "$OUT/drag_L_${STAMP}.csv" "$DUR" &
pids+=($!)
# 错开连接，减轻 SDK 日志竞态
sleep 0.5
"$BIN" "$RIGHT" "$LOCAL" "$OUT/drag_R_${STAMP}.csv" "$DUR" &
pids+=($!)

wait "${pids[@]}" || true
echo "done. CSV under $OUT/drag_{L,R}_${STAMP}.csv"
