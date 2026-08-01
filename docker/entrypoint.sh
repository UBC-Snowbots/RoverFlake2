#!/bin/bash
set -e

# source system ros2
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

# build
if [ ! -f "$ROVERFLAKE_ROOT/install/setup.bash" ]; then
  echo "No install/setup.bash found, running colcon build..."
  if ! colcon build; then # should automatically use --symlink-install
    echo "WARNING: initial colcon build failed. Container stays up so you can debug." >&2
    if [ ! -f "$ROVERFLAKE_ROOT/src/external_pkgs/moteus/CMakeLists.txt" ]; then
      echo "HINT: git submodules look uninitialized. On the host run: git submodule update --init --recursive" >&2
    fi
  fi
fi

# Source workspace overlay
if [ -f "$ROVERFLAKE_ROOT/install/setup.bash" ]; then
  source "$ROVERFLAKE_ROOT/install/setup.bash"
fi

exec "$@"
