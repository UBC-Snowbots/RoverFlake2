source $HOME/RoverFlake2/setup_scripts/utils/common.sh

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


apt_packages_to_install=(
    "curl"
    "git"
    "btop"
    "tmux"
    
    # add more packages as needed
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