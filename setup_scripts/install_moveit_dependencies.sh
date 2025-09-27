source $HOME/RoverFlake2/setup_scripts/utils/common.sh
echo script not ready yet && false


# apt_packages_to_install=(
#     "ros-humble-moveit-"
#     "git"
#     "btop"
#     "tmux"
#     "libgtkmm-3.0-dev"
# )



# # Loop through the package list and install missing packages
# for package in "${apt_packages_to_install[@]}"; do
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