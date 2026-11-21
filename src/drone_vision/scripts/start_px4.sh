#!/usr/bin/env bash
set -eo pipefail
: "${PX4_SITL_ROOT:?Set PX4_SITL_ROOT to the PX4 source checkout}"
root=$(cd -- "$PX4_SITL_ROOT" && pwd)
build="$root/build/px4_sitl_default"
[[ -x "$build/bin/px4" ]] || { echo 'Existing PX4 SITL build missing.' >&2; exit 1; }
export PX4_SIM_MODEL=gazebo-classic_iris
mkdir -p "$build/rootfs"
cd "$build/rootfs"
exec "$build/bin/px4" -d "$build/etc"
