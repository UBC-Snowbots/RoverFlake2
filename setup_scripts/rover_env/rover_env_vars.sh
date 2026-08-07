# All our common enviroment variables
export COLCON_DEFAULTS_FILE="${ROVERFLAKE_ROOT}/colcon.defaults.yaml"

# Rover operational DDS domain. Pre-exported values win (e.g. bench isolation).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:101}"

export RANDOM_INSTALL_FILE_DIR="${ROVERFLAKE_ROOT}/random_install_files"