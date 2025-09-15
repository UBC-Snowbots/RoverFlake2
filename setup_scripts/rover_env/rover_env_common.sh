# During setup, this file is added to your .bashrc via a single line (source rover_env_common.sh) similar to how /opt/ros/humble/setup.bash is added
source ${ROVERFLAKE_ROOT}/setup_scripts/rover_env/rover_aliases_common.sh #Aliases like rosbuild, rosclean etc
source ${ROVERFLAKE_ROOT}/setup_scripts/rover_env/rover_env_vars.sh
source ${ROVERFLAKE_ROOT}/install/setup.bash # Default to sourcing the repo (May mess up if you have multiple ROS2 or ROS repos)