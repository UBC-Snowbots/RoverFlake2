#!/bin/bash
set -e

# source system ros2
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# build
if [ ! -f "$ROVERFLAKE_ROOT/install/setup.bash" ]; then
  echo "No install/setup.bash found, running colcon build..."
  colcon build # should automatically use --symlink-install
fi

# Source workspace overlay
if [ -f "$ROVERFLAKE_ROOT/install/setup.bash" ]; then
  source "$ROVERFLAKE_ROOT/install/setup.bash"
fi

exec "$@"
