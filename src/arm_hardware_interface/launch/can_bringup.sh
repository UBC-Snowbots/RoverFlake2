#!/usr/bin/env bash
# Brings a SocketCAN interface up with the settings from config/can_loop.yaml.
#
# Bitrate and bit timing belong to the network interface, not to our process, so
# they get set with `ip link`. That needs root - either run this yourself once
# after plugging in the CANable, or let the launch file run it (see the
# bring_up_interface launch argument, which needs passwordless sudo).
#
# Usage:
#   ./can_setup.sh --interface can0 --bitrate 500000 [options]
#
# Options:
#   --interface NAME          interface name, ie can0            (required)
#   --bitrate BPS             arbitration bitrate                (required)
#   --sample-point 0.875      arbitration sample point
#   --fd                      enable CAN FD
#   --data-bitrate BPS        data phase bitrate (CAN FD only)
#   --data-sample-point 0.75  data phase sample point (CAN FD only)
#   --restart-ms MS           auto restart after bus-off, 0 to disable
#   --txqueuelen N            kernel tx queue length

set -euo pipefail

INTERFACE=""
BITRATE=""
SAMPLE_POINT=""
USE_FD="false"
DATA_BITRATE=""
DATA_SAMPLE_POINT=""
RESTART_MS=""
TXQUEUELEN=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --interface)         INTERFACE="$2";         shift 2 ;;
        --bitrate)           BITRATE="$2";           shift 2 ;;
        --sample-point)      SAMPLE_POINT="$2";      shift 2 ;;
        --fd)                USE_FD="true";          shift 1 ;;
        --data-bitrate)      DATA_BITRATE="$2";      shift 2 ;;
        --data-sample-point) DATA_SAMPLE_POINT="$2"; shift 2 ;;
        --restart-ms)        RESTART_MS="$2";        shift 2 ;;
        --txqueuelen)        TXQUEUELEN="$2";        shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

if [[ -z "$INTERFACE" || -z "$BITRATE" ]]; then
    echo "--interface and --bitrate are required" >&2
    exit 1
fi

# Use sudo if we are not already root.
SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
    SUDO="sudo"
fi

if [[ ! -d "/sys/class/net/${INTERFACE}" ]]; then
    echo "Interface ${INTERFACE} does not exist. Is the CANable plugged in?" >&2
    exit 1
fi

# Build the `ip link set type can` arguments.
CAN_ARGS=(bitrate "$BITRATE")
[[ -n "$SAMPLE_POINT" ]] && CAN_ARGS+=(sample-point "$SAMPLE_POINT")

if [[ "$USE_FD" == "true" ]]; then
    [[ -n "$DATA_BITRATE" ]]      && CAN_ARGS+=(dbitrate "$DATA_BITRATE")
    [[ -n "$DATA_SAMPLE_POINT" ]] && CAN_ARGS+=(dsample-point "$DATA_SAMPLE_POINT")
    CAN_ARGS+=(fd on)
fi

[[ -n "$RESTART_MS" ]] && CAN_ARGS+=(restart-ms "$RESTART_MS")

# The interface has to be down before the timing can be changed.
$SUDO ip link set "$INTERFACE" down || true
$SUDO ip link set "$INTERFACE" type can "${CAN_ARGS[@]}"

if [[ -n "$TXQUEUELEN" ]]; then
    $SUDO ip link set "$INTERFACE" txqueuelen "$TXQUEUELEN"
fi

$SUDO ip link set "$INTERFACE" up

echo "${INTERFACE} is up:"
ip -details -statistics link show "$INTERFACE" | head -n 6