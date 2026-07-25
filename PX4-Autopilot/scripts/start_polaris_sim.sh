#!/usr/bin/env bash
# Start one PX4/Gazebo instance with a shared loopback transport configuration.
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
polaris_dir=$(cd "$root_dir/.." && pwd)
build_dir="$root_dir/build/px4_sitl_default"
runtime_dir="$root_dir/.polaris-sim"
pid_file="$runtime_dir/gz-server.pid"
gui_pid_file="$runtime_dir/gz-gui.pid"

# Camera-equipped PX4 simulation target.
sim_target="gz_x500_mono_cam"
selected_world=${POLARIS_WORLD:-default}

case "$selected_world" in
	default)
		world_name="default"
		world_file="$root_dir/Tools/simulation/gz/worlds/default.sdf"
		;;
	antarctica)
		world_name="antarctica"
		world_file="$polaris_dir/simulation/worlds/polaris_antarctica.sdf"
		;;
	*)
		printf 'Unknown POLARIS_WORLD=%s. Supported worlds: default, antarctica\n' \
			"$selected_world" >&2
		exit 1
		;;
esac

if [[ ! -x "$build_dir/bin/px4" || ! -f "$build_dir/rootfs/gz_env.sh" ]]; then
	printf 'PX4 SITL is not built. Run: make px4_sitl %s\n' "$sim_target" >&2
	exit 1
fi

if [[ ! -f "$world_file" ]]; then
	printf 'Gazebo world file not found: %s\n' "$world_file" >&2
	exit 1
fi

if pgrep -f 'gz sim' >/dev/null; then
	printf 'A Gazebo instance is already running. Stop it before starting Polaris SIM.\n' >&2
	exit 1
fi

mkdir -p "$runtime_dir"
rm -f "$pid_file" "$gui_pid_file"

# PX4 uses GZ_IP=127.0.0.1 for the Gazebo target.
# The server, GUI, and PX4 process must share the same transport configuration.
unset GZ_TRANSPORT_LOCALHOST_ONLY
unset IGN_TRANSPORT_DISABLE_MULTICAST
unset IGN_IP

export GZ_IP=127.0.0.1
export DYLD_FALLBACK_LIBRARY_PATH="${DYLD_FALLBACK_LIBRARY_PATH:-}"

# PX4's generated gz_env.sh expects these variables to exist under `set -u`.
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PLUGIN_PATH="${GZ_PLUGIN_PATH:-}"

# shellcheck disable=SC1091
source "$build_dir/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="$polaris_dir/simulation/models:$polaris_dir/simulation/worlds:$GZ_SIM_RESOURCE_PATH"

cleanup() {
	"$root_dir/scripts/stop_polaris_sim.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# On macOS, run the Gazebo server separately from the GUI.
gz sim -r -s -v 3 "$world_file" \
	>"$runtime_dir/gazebo-server.log" 2>&1 &

server_pid=$!
printf '%s\n' "$server_pid" >"$pid_file"

for _ in {1..20}; do
	if ! kill -0 "$server_pid" 2>/dev/null; then
		printf 'Gazebo server exited unexpectedly. See %s\n' \
			"$runtime_dir/gazebo-server.log" >&2
		exit 1
	fi

	if gz service -l | /usr/bin/grep -qx "/world/$world_name/scene/info"; then
		break
	fi

	sleep 1
done

if ! gz service -l | /usr/bin/grep -qx "/world/$world_name/scene/info"; then
	printf 'Gazebo did not become ready. See %s\n' \
		"$runtime_dir/gazebo-server.log" >&2
	exit 1
fi

if [[ ${POLARIS_SIM_HEADLESS:-0} != 1 ]]; then
	# Gazebo Harmonic on macOS requires the GUI as a separate client process.
	gz sim -g >"$runtime_dir/gazebo-gui.log" 2>&1 &
	gui_pid=$!
	printf '%s\n' "$gui_pid" >"$gui_pid_file"
fi

printf 'Gazebo world %s is ready. Starting PX4 model %s; logs: %s\n' \
	"$world_name" "$sim_target" "$runtime_dir"

PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world_name" \
	make -C "$root_dir" px4_sitl "$sim_target" \
	2>&1 | tee "$runtime_dir/px4.log"
