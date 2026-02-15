source $ROVERFLAKE_ROOT/setup_scripts/utils/common.sh

echo CHECKING FOR ROS2 DESKTOP
if is_package_installed "ros-humble-desktop"; then
    echo ROS HUMBLE FOR $USER IS ALREADY INSTALLED
else
    locale  # check for UTF-8

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify setttings

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe

    # add keys and sources so we can use apt to install everything
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    

    sudo apt update
    sudo apt upgrade -y

    # now we have ros2 apt packages. celebrate this.
    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-dev-tools

    echo source /opt/ros/humble/setup.bash >> ~/.bashrc
    source ~/.bashrc
fi

echo install-ros2-humble.sh complete.
