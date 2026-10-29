#!/usr/bin/env bash
task_dds_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
  rmw_fastrtps_cpp) ;;
  *) printf 'The RMUL point-cloud transport profile requires rmw_fastrtps_cpp\n' >&2; return 2 ;;
esac
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${RMUL2027_DDS_PROFILE:-$task_dds_dir/config/fastdds_local_udp_4m.xml}"
if [ ! -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  printf 'Missing local DDS profile: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
  return 1
fi
printf 'Project-local DDS profile: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE"
