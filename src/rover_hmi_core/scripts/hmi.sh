#!/usr/bin/env bash
# hmi.sh — launch the Rover HMI with workspace, env, and domain set up.
# All paths derive from this script's location, so the same script works
# from the main checkout or any worktree. Run inside the rover container.
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT/install/setup.bash" ]; then
    echo "hmi.sh: no install/ under $ROOT — build first:" >&2
    echo "  cd $ROOT && colcon build --packages-up-to rover_hmi_core" >&2
    exit 1
fi
source "$ROOT/install/setup.bash"

export ROVERFLAKE_ROOT="$ROOT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"

exec ros2 run rover_hmi_core rover_hmi "$@"
