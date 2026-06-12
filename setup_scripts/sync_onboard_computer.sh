help() {
    echo "Usage:"
    echo "./sync_onboard_computer.sh user@destination-ip"
    echo "For USB connections, find the destination ip using `ip route | grep usb`"
    exit 1
}

if [ -z "$1" ]; then
    help
fi

REMOTE_TARGET="$1"

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

REMOTE_DST="/home/roverflake/"

echo "Transferring files to $REMOTE_TARGET..."
rsync -azc --delete --stats ../install "$REMOTE_TARGET:$REMOTE_DST"
echo "Transfer complete!"