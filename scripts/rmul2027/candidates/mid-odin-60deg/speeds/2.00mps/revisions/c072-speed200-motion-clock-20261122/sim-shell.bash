#!/usr/bin/env bash
set -euo pipefail
runtime=/tmp/rmul-jammy/root
archive=/tmp/rmul-original-20261119
results=/home/yq/Downloads/rmul2027-taildrop-20261119/original-60deg-iterations-20261119
source_root=/home/hardy/pb2025_nav_ws
outputs=/mnt/c/Users/Hardy/Documents/Codex/2026-10-18/headoff-md1/outputs
threads=${RMUL_SOFTWARE_THREADS:-4}
lidar_only=${RMUL_NAV_SENSOR_ONLY:-0}
render_lod=${RMUL_NAV_RENDER_LOD:-0}
[[ "$threads" =~ ^([2-9]|1[0-6])$ && "$lidar_only" =~ ^[01]$ ]]
graphics=()
if [[ ${RMUL_HEADLESS_NVIDIA:-0} == 1 ]]; then
    graphics+=(--ro-bind /tmp/rmul-jammy/nvidia-driver /tmp/rmul-nvidia
      --setenv LD_LIBRARY_PATH /tmp/rmul-nvidia
      --setenv __EGL_VENDOR_LIBRARY_FILENAMES /tmp/rmul-nvidia/10_nvidia.json
      --setenv RMUL_HEADLESS_NVIDIA 1)
    for device in /dev/nvidia0 /dev/nvidiactl /dev/nvidia-uvm; do
        graphics+=(--dev-bind "$device" "$device")
    done
fi
exec env -i /usr/bin/bwrap --unshare-user --uid 0 --gid 0 --unshare-net --unshare-pid --unshare-ipc --unshare-uts --die-with-parent \
  --ro-bind "$runtime" / --proc /proc --dev /dev --tmpfs /dev/shm --tmpfs /tmp \
  --tmpfs /home/sim --bind "$archive/workspace" "$source_root" \
  --ro-bind "$(dirname -- "${BASH_SOURCE[0]}")/instrumentation/world.sdf" "$source_root/src/pb2025_sentry_nav/scripts/rmul2027/scene/worlds/rmul_2027_3v3_flat_floor_world.sdf" \
  --ro-bind "$(dirname -- "${BASH_SOURCE[0]}")/instrumentation/field.sdf" "$source_root/src/pb2025_sentry_nav/scripts/rmul2027/scene/models/rmul_2027_3v3_flat_floor/model.sdf" \
  --ro-bind "$archive/outputs" "$outputs" --bind "$results" /evidence \
  --chdir "$source_root" --setenv PATH /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  --setenv HOME /home/sim --setenv TZ Asia/Shanghai --setenv LC_ALL C.UTF-8 \
  --setenv RMUL_WORKSPACE "$source_root" --setenv RMUL_STORAGE_HOST_ROOT /evidence \
  --setenv ROS_LOCALHOST_ONLY 1 --setenv ROS_DOMAIN_ID 195 \
  --setenv IGN_IP 127.0.0.1 --setenv LP_NUM_THREADS "$threads" --setenv RMUL_NAV_SENSOR_ONLY "$lidar_only" --setenv RMUL_NAV_RENDER_LOD "$render_lod" --setenv EGL_PLATFORM surfaceless \
  --setenv OMP_NUM_THREADS 2 --setenv OPENBLAS_NUM_THREADS 1 --setenv PYTHONDONTWRITEBYTECODE 1 \
  --setenv RMUL_ARCHIVE_DIGEST "$(cat /tmp/rmul-jammy/archive-sha256.txt)" \
  "${graphics[@]}" \
  /bin/bash "$@"
