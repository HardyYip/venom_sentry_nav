#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_field="${1:-cad-flat-floor}"
task_render="${2:-auto}"
case "$task_field" in cad|cad-flat-marking|cad-flat-floor|seam-ramps) ;; *) printf 'Expected cad, cad-flat-marking, cad-flat-floor or seam-ramps\n' >&2; exit 2 ;; esac
case "$task_render" in auto|gpu|software) ;; *) printf 'Expected auto, gpu or software\n' >&2; exit 2 ;; esac
if [ "$#" -gt 2 ]; then printf 'Expected scene and optional rendering mode\n' >&2; exit 2; fi
export RMUL2027_DDS_PROFILE="${RMUL2027_DDS_PROFILE:-$task_dir/config/fastdds_local_64m.xml}"
source "$task_dir/configure_dds.sh"
printf '2.5 m/s profile, 2m approach slowdown, project-local 64MiB DDS transport.\n'
exec bash "$task_dir/start_rmul2027.sh" nav \
  --render-mode "$task_render" --field-contact "$task_field" \
  --gz-world-path "$task_dir/config/gz_world_test.yaml" \
  --params-file "$task_dir/config/nav2_original_speed_early_brake.yaml" \
  --tf2-library-dir "$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib"
