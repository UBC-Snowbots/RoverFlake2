
# ? we can do a script dir, but easier and more explicit just to get the user to define their repo root
# SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo $ROVERFLAKE_ROOT

# check if VAR is unset or empty
if [ -z "${ROVERFLAKE_ROOT}" ]; then
  echo "ROVERFLAKE_ROOT is not set (or empty). That makes me really sad."
  sleep 2
  echo "Please set the enviroment variable in your bashrc to the root of RoverFlake2/"
  exit 1
fi


source $ROVERFLAKE_ROOT/utils/common.sh

clear

echo STARTING SETUP
for ((i = 0; i < 100; i++)); do 
    echo -n "#"
    sleep 0.01
done
clear
sleep 0.5
echo Hello $USER, Welcome to the team. 
# sleep 2
echo This script will install all commonly used tools in our main workflow.
# sleep 1
echo Will install ros2 humble, vscode, btop, tmux and other useful tools.
# sleep 1
echo This script will NOT install discord. run install-discord.sh if you want discord.
sleep 1.0
echo starting with ros2...

#sudo apt update
cd $ROVERFLAKE_ROOT/setup_scripts/
bash install_dependencies.sh
bash install-ros2-humble.sh # also runs apt update, if ros2 is not installed

apt_packages_to_install=(
    "curl"
    "git"
    "btop"
    "tmux"
    "libgtkmm-3.0-dev"
    "python3-rosdep"
    "libsfml-dev"
)



# Loop through the package list and install missing packages
for package in "${apt_packages_to_install[@]}"; do
    if is_package_installed "$package"; then
        echo "Package '$package' is already installed."
    else
        echo "Installing package '$package'..."
        # sudo apt update
        sudo apt install -y "$package"
        
        if [ $? -eq 0 ]; then
            echo "Package '$package' installed successfully."
        else
            echo "Failed to install package '$package'."
        fi
    fi
done
cd $ROVERFLAKE_ROOT

source /opt/ros/humble/setup.bash
#install ros2 packages
sudo rosdep init
rosdep update
bash setup_scripts/install_rosdeps.sh

# setup user enviroment
mkdir ${ROVERFLAKE_ROOT}/random_install_files
echo "source ${ROVERFLAKE_ROOT}/setup_scripts/rover_env/rover_env_common.sh " >> ~/.bashrc


# install libsweep
bash ${ROVERFLAKE_ROOT}/setup_scripts/install_libsweepsdk.sh


cd ${ROVERFLAKE_ROOT}
bash setup_scripts/submodule_update.sh




# for package in "${ros_packages_to_install[@]}"; do
#     if is_package_installed "$package"; then
#         echo "Package '$package' is already installed."
#     else
#         echo "Installing package '$package'..."
#         # sudo apt update
#         sudo apt install -y "$package"
        
#         if [ $? -eq 0 ]; then
#             echo "Package '$package' installed successfully."
#         else
#             echo "Failed to install package '$package'."
#         fi
#     fi
# done
