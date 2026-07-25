#!/usr/bin/env bash
# Stop PX4 and Gazebo processes started by start_polaris_sim.sh.
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
runtime_dir="$root_dir/.polaris-sim"
px4_rootfs="$root_dir/build/px4_sitl_default/rootfs"

if [[ -S /tmp/px4-sock-0 && -x "$root_dir/build/px4_sitl_default/bin/px4-shutdown" ]]; then
	(
		cd "$px4_rootfs"
		../bin/px4-shutdown
	) >/dev/null 2>&1 || true
fi

for pid_file in "$runtime_dir/gz-gui.pid" "$runtime_dir/gz-server.pid"; do
	if [[ -f "$pid_file" ]]; then
		pid=$(<"$pid_file")
		if kill -0 "$pid" 2>/dev/null; then
			kill -TERM "$pid"
		fi
		rm -f "$pid_file"
	fi
done
