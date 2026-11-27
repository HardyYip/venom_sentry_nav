#!/bin/bash
set -euo pipefail
manual=$(cd -- "$(dirname -- "$0")" && pwd)
here=$(cd -- "$manual/../native-nav" && pwd)
baseline=$(dirname "$(dirname "$here")")/c072-verify-headon-planar-accel-20261123
runtime=${ODIN_RUNTIME:-$here/../runtime}
viewer=${CART_RVIZ_PREFIX:-$runtime/root/opt/ros/humble}
auth=${XAUTHORITY:-$HOME/.Xauthority}
: "${DISPLAY:?Set DISPLAY to the local X11 desktop}"
for dep in "$viewer/lib/rviz2/rviz2" "$auth" "$baseline/planner-install" "$runtime/root" "$here/safety-install/lib/librmul_pose_fresh_condition_bt_node.so"; do
    if [[ ! -e "$dep" ]]; then echo "Missing dependency: $dep" >&2; exit 1; fi
done
session=$(mktemp -d "${TMPDIR:-/tmp}/cart-rviz-200.XXXXXXXX")
cp -- "$here/full-rerun-1795757044818/left-near-01/"{world.sdf,nav.yaml,case.json} "$session/"
cp -- "$manual/"{cart-body.rviz,cart.urdf,manual_fixture.py,run-manual.bash} "$session/"
echo "RViz2 session logs: $session"
devices=()
for dev in nvidiactl nvidia0 nvidia-uvm nvidia-modeset; do
    if test -c "/dev/$dev"; then devices+=(--dev-bind "/dev/$dev" "/dev/$dev"); fi
done
exec env -i bwrap --unshare-user --uid 0 --gid 0 --unshare-pid --unshare-net --unshare-ipc --unshare-uts --die-with-parent \
 --ro-bind "$runtime/root" / --dev /dev --proc /proc --tmpfs /tmp --tmpfs /dev/shm --dir /tmp/home \
 --ro-bind "$runtime/omni" /omni --ro-bind "$runtime/pb_nav2_plugins" /pbnav --ro-bind "$runtime/field-models" /field-models --ro-bind "$baseline" /baseline --ro-bind "$runtime/gfx" /gfx --ro-bind "$here" /trial --bind "$session" /tmp/manual --ro-bind "$viewer" /tmp/viewer --ro-bind /tmp/.X11-unix /tmp/.X11-unix --ro-bind "$auth" /tmp/cart-xauth --setenv CART_DISPLAY "$DISPLAY" "${devices[@]}" \
 --setenv HOME /tmp/home --setenv PATH /usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin --setenv LANG C.UTF-8 \
 --setenv ROS_LOCALHOST_ONLY 1 --setenv ROS_DOMAIN_ID 177 --setenv IGN_IP 127.0.0.1 --setenv EGL_PLATFORM surfaceless \
 --setenv LD_LIBRARY_PATH /gfx --setenv __EGL_VENDOR_LIBRARY_FILENAMES /gfx/10_nvidia.json --setenv OMP_NUM_THREADS 1 --setenv OPENBLAS_NUM_THREADS 1 --chdir /trial \
 /bin/bash -c 'source /opt/ros/humble/setup.bash; source /omni/local_setup.bash; source /baseline/planner-install/local_setup.bash; export AMENT_PREFIX_PATH=/pbnav:$AMENT_PREFIX_PATH LD_LIBRARY_PATH=/trial/safety-install/lib:/pbnav/lib:$LD_LIBRARY_PATH; exec "$@"' isolated /bin/bash /tmp/manual/run-manual.bash
