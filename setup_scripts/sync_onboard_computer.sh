help() {
    echo "Usage: ./sync_onboard_computer.sh [network | usb /path/to/usb] "
    exit 1
}

if [ -z "$1" ]; then
    help
fi

TARGET="$1"
if [ "$TARGET" == "usb" ]; then
    if [ -z "$2" ]; then
        help
    fi
    USB_PATH="$2"
fi

PWD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVERFLAKE_ROOT="$(cd "$PWD/.." && pwd)"
echo "Roverflake project root set to $ROVERFLAKE_ROOT"
cd "$ROVERFLAKE_ROOT/src" || exit 1

echo "Compiling packages locally..."
MAKEFLAGS="-j3" colcon build  --build-base ../build --install-base ../install --symlink-install
if [ $? -ne 0 ]; then
    echo "Compilation failed. File transfer aborted"
    exit 1
fi

if [ "$TARGET" == "usb" ]; then
    if [ ! -d "$USB_PATH" ]; then
        echo "Invalid USB path: $USB_PATH"
    fi

    DST="$USB_PATH/Test_Onboard_Sync"
    echo "Transferring files to $DST..."
    mkdir -p "$DST"

    rsync -rcLptvz --modify-window=1 --stats ../install "$DST/"

    echo "Transfer complete!"
elif [ "$TARGET" == "network" ]; then
    echo "Network function not implemented yet, please use USB mode"
else
    help
fi