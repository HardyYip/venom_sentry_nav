#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_render="${1:-auto}"
case "$task_render" in auto|gpu|software) ;; *) printf 'Usage: bash %s [auto|gpu|software]\n' "$0" >&2; exit 2 ;; esac
if [ "$#" -gt 1 ]; then printf 'Expected at most one rendering argument\n' >&2; exit 2; fi
source /opt/ros/humble/setup.bash
source "$task_ws/install/setup.bash"
source "$task_dir/configure_dds.sh"
exec 8>"$task_ws/.rmul2027-rviz.lock"
if ! flock -n 8; then printf 'A project RViz is already running. Close it before reopening.\n' >&2; exit 1; fi
export QT_QPA_PLATFORM=xcb
if [ "$task_render" = software ]; then
  export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=2
  printf 'RViz renderer: software (explicit llvmpipe fallback)\n'
else
  unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
  task_probe=
  if command -v glxinfo >/dev/null 2>&1; then
    task_probe="$(timeout 10 glxinfo -B 2>&1 || true)"
    if ! [[ "$task_probe" == *'Accelerated: yes'* ]] && [ -e /dev/dxg ]; then
      export GALLIUM_DRIVER=d3d12
      export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
      task_probe="$(timeout 10 glxinfo -B 2>&1 || true)"
    fi
    if [[ "$task_probe" == *'Accelerated: yes'* ]]; then
      printf '%s\n' "$task_probe" | sed -n '/Accelerated:/p; /OpenGL renderer string:/p'
    elif [ "$task_render" = gpu ]; then
      printf 'Hardware OpenGL probe failed; RViz was not started.\n%s\n' "$task_probe" >&2
      exit 1
    else
      export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=2
      printf 'Hardware OpenGL unavailable; RViz is using software rendering.\n' >&2
    fi
  elif [ "$task_render" = gpu ]; then
    printf 'Install mesa-utils to verify hardware OpenGL before using gpu mode.\n' >&2
    exit 1
  else
    printf 'glxinfo unavailable; using the system renderer without forcing software.\n' >&2
  fi
fi
if [ -d /mnt/wslg ] && command -v powershell.exe >/dev/null 2>&1 && [ "${RMUL2027_RESTORE_WINDOW:-1}" != 0 ]; then
  powershell.exe -NoLogo -NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass \
    -File "$(wslpath -w "$task_dir/restore_rviz_window.ps1")" \
    -ConfigTitle "$task_dir/config/interactive-light.rviz" 8>&- &
fi
exec ros2 run rviz2 rviz2 -d "$task_dir/config/interactive-light.rviz" \
  --ros-args -r __ns:=/red_standard_robot1 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=true
