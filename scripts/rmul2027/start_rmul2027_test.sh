#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ws="$(cd -- "$task_dir/../../../.." && pwd)"
task_field="${1:-cad}"
case "$task_field" in cad|seam-ramps) ;; *) printf 'Usage: bash %s [cad|seam-ramps]\n' "$0" >&2; exit 2 ;; esac
if [ "$#" -gt 1 ]; then printf 'Expected at most one scene argument\n' >&2; exit 2; fi
exec bash "$task_dir/start_rmul2027.sh" nav \
  --field-contact "$task_field" --gz-world-path "$task_dir/config/gz_world_test.yaml" \
  --params-file "$task_dir/config/nav2_test.yaml" \
  --tf2-library-dir "$task_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib"
