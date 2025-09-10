source $REPO_ROOT/setup_scripts/utils/common.sh

echo CHECKING FOR ROS2 DESKTOP
if is_package_installed "ros-humble-desktop"; then
    echo ROS HUMBLE FOR $USER IS ALREADY INSTALLED
else
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify setttings

    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    sudo apt upgrade

    sudo apt install ros-humble-desktop

    sudo apt install ros-dev-tools

    echo source /opt/ros/humble/setup.bash >> ~/.bashrc
fi

echo install-ros2-humble.sh complete.
