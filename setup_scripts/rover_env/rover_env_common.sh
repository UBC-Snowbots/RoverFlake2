# During setup, this file is added to your .bashrc via a single line (source rover_env_common.sh) similar to how /opt/ros/humble/setup.bash is added
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash # Underlay must come first, install/setup.bash alone breaks on a clean workspace
fi
source ${ROVERFLAKE_ROOT}/setup_scripts/rover_env/rover_aliases_common.sh #Aliases like rosbuild, rosclean etc
source ${ROVERFLAKE_ROOT}/setup_scripts/rover_env/rover_env_vars.sh
# Guarded for docker: a fresh container/clean workspace has no install/ yet
if [ -f ${ROVERFLAKE_ROOT}/install/setup.bash ]; then
  source ${ROVERFLAKE_ROOT}/install/setup.bash # Default to sourcing the repo (May mess up if you have multiple ROS2 or ROS repos)
fi