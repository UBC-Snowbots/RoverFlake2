
# ? we can do a script dir, but easier and more explicit just to get the user to define their repo root
# SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo $ROVERFLAKE_ROOT

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
exit 1

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
rosdep install --from-paths src -y --ignore-src
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
