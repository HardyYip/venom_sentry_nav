#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_field="${1:-cad-flat-floor}"
task_render="${2:-auto}"
case "$task_field" in cad|cad-flat-marking|cad-flat-floor|seam-ramps) ;; *) printf 'Usage: bash %s [cad|cad-flat-marking|cad-flat-floor|seam-ramps] [software|gpu|auto]\n' "$0" >&2; exit 2 ;; esac
case "$task_render" in software|gpu|auto) ;; *) printf 'Expected rendering mode software, gpu or auto\n' >&2; exit 2 ;; esac
if [ "$#" -gt 2 ]; then printf 'Expected scene and optional rendering mode\n' >&2; exit 2; fi
exec bash "$task_dir/start_rmul2027.sh" nav \
  --render-mode "$task_render" --field-contact "$task_field" --gz-world-path "$task_dir/config/gz_world_test.yaml" \
  --params-file "$task_dir/config/nav2_test.yaml" \
  --tf2-library-dir "$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib"
