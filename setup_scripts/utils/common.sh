
# export ROVERFLAKE_INSTALL_SCRIPTS_UTILS_SOURCED="true"

is_package_installed(){
        local package_name="$1"
    
    # Check if the package is installed using dpkg-query
    if dpkg-query -W -f='${Status}' "$package_name" 2>/dev/null | grep -q "installed"; then
        # Get the package version
        package_version=$(dpkg-query -W -f='${Version}' "$package_name")
        echo "Package '$package_name' is already installed. Version: $package_version"
    else
        echo "Package '$package_name' is not installed."
    fi
}

is_snap_package_installed() {
    local snap_name="$1"

    # Use snap list to check if the package is installed
    if snap list | grep -q "^$snap_name\s"; then
        echo "Snap package '$snap_name' is already installed."
        return 0
    else
        echo "Snap package '$snap_name' is not installed."
        return 1
    fi
}