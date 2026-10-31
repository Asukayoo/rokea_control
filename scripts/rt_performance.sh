#!/usr/bin/env bash
# 1 kHz 测前：performance + IRQ 挪出 RT 核 + 压住 C-state（cpu_dma_latency）。
# 用法：
#   sudo bash scripts/rt_performance.sh          # on
#   sudo bash scripts/rt_performance.sh on
#   sudo bash scripts/rt_performance.sh off      # powersave
#
# 环境变量 RT_CPUS 默认 2,4（须与 grub isolcpus 一致）
set -euo pipefail

mode="${1:-on}"
RT_CPUS="${RT_CPUS:-2,4}"
DMA_LAT_HOLD="/run/rokea-cpu-dma-latency.hold"

case "$mode" in
  on|performance|perf) gov=performance ;;
  off|powersave|ps)    gov=powersave ;;
  *)
    echo "usage: $0 [on|off]" >&2
    exit 2
    ;;
esac

if [[ "$(id -u)" -ne 0 ]]; then
  echo "need root: sudo $0 $mode" >&2
  exit 1
fi

n=0
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [[ -f "$g" ]] || continue
  echo "$gov" > "$g"
  n=$((n + 1))
done
echo "governor=$gov ($n cpus) RT_CPUS=$RT_CPUS"

# 生成管家核列表（online \ RT_CPUS）→ affinity 字符串
build_allow_list() {
  local -A is_rt=()
  local c
  IFS=',' read -r -a rt <<< "$RT_CPUS"
  for c in "${rt[@]}"; do is_rt["$c"]=1; done
  local allow=()
  local nums=()
  for d in /sys/devices/system/cpu/cpu[0-9]*; do
    c=${d##*/cpu}
    [[ "$c" =~ ^[0-9]+$ ]] || continue
    [[ -n "${is_rt[$c]:-}" ]] && continue
    nums+=("$c")
  done
  # 数值排序，避免 cpu10 插在 cpu1 后
  mapfile -t allow < <(printf '%s\n' "${nums[@]}" | sort -n)
  local out="" start="" prev=""
  for c in "${allow[@]}"; do
    if [[ -z "$start" ]]; then
      start=$c; prev=$c; continue
    fi
    if [[ "$c" -eq $((prev + 1)) ]]; then
      prev=$c; continue
    fi
    if [[ "$start" == "$prev" ]]; then out+="${start},"; else out+="${start}-${prev},"; fi
    start=$c; prev=$c
  done
  if [[ -n "$start" ]]; then
    if [[ "$start" == "$prev" ]]; then out+="$start"; else out+="${start}-${prev}"; fi
  fi
  echo "$out"
}

cpu_in_list() {
  # $1=cpu  $2=affinity_list like 2-3 or 0-1,3,5-15
  local cpu=$1 list=$2 tok a b
  IFS=',' read -r -a parts <<< "$list"
  for tok in "${parts[@]}"; do
    if [[ "$tok" == *-* ]]; then
      a=${tok%-*}; b=${tok#*-}
      [[ "$cpu" -ge "$a" && "$cpu" -le "$b" ]] && return 0
    else
      [[ "$tok" == "$cpu" ]] && return 0
    fi
  done
  return 1
}

if [[ "$gov" == performance ]]; then
  aff=$(build_allow_list)
  echo "irq_allow=$aff"
  moved=0
  IFS=',' read -r -a rt <<< "$RT_CPUS"
  for irq_dir in /proc/irq/[0-9]*; do
    [[ -f "$irq_dir/smp_affinity_list" ]] || continue
    cur=$(cat "$irq_dir/smp_affinity_list" 2>/dev/null || true)
    [[ -n "$cur" ]] || continue
    touch=0
    for c in "${rt[@]}"; do
      if cpu_in_list "$c" "$cur"; then touch=1; break; fi
    done
    [[ "$touch" -eq 1 ]] || continue
    if echo "$aff" > "$irq_dir/smp_affinity_list" 2>/dev/null; then
      moved=$((moved + 1))
    else
      # managed IRQ（如 nvme 队列）在 isolcpus=managed_irq 生效前常拒绝改写
      :
    fi
  done
  echo "irq_moved=$moved (managed IRQs need isolcpus=managed_irq after reboot)"

  # 压 C-state：写 0 并保持 fd（后台）
  if [[ -e /dev/cpu_dma_latency ]]; then
    if [[ -e "$DMA_LAT_HOLD" ]] && kill -0 "$(cat "$DMA_LAT_HOLD" 2>/dev/null)" 2>/dev/null; then
      echo "cpu_dma_latency: already held by pid $(cat "$DMA_LAT_HOLD")"
    else
      (
        exec >/dev/null 2>&1
        exec 3>/dev/cpu_dma_latency
        printf '\0\0\0\0' >&3
        echo $BASHPID > "$DMA_LAT_HOLD"
        while true; do sleep 3600; done
      ) &
      disown 2>/dev/null || true
      sleep 0.05
      echo "cpu_dma_latency: held (pid $(cat "$DMA_LAT_HOLD" 2>/dev/null || echo ?))"
    fi
  fi
else
  if [[ -e "$DMA_LAT_HOLD" ]]; then
    pid=$(cat "$DMA_LAT_HOLD" 2>/dev/null || true)
    if [[ -n "${pid:-}" ]]; then
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$DMA_LAT_HOLD"
    echo "cpu_dma_latency: released"
  fi
fi

for c in 0 2 4; do
  f="/sys/devices/system/cpu/cpu${c}/cpufreq/scaling_governor"
  [[ -f "$f" ]] && echo "  cpu$c=$(cat "$f")"
done
isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)
if [[ -n "$isolated" ]]; then
  echo "cpu/isolated=\"${isolated}\" OK"
else
  echo "cpu/isolated=\"\" (重启后 isolcpus 才生效)"
fi
