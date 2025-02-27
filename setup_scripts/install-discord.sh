source $HOME/RoverFlake2/setup_scripts/utils/common.sh

if is_snap_package_installed "discord"; then
    echo discord is already installed
else
    echo discord is not installed, installing with snap
    sudo snap install discord
    echo complete
fi