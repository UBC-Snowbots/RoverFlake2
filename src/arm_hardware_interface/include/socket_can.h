// Thin wrapper around a SocketCAN raw socket.
// Knows nothing about ROS2 or the comms library - it just moves frames.
// Bringing the interface up (bitrate, timing, fd) is NOT done here, see scripts/can_setup.sh.
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <string>

// We always hand around canfd_frame, even for classic CAN. A classic frame is
// just a canfd_frame that never uses more than 8 data bytes, and the first few
// fields (can_id, len) sit at the same offsets in both structs.
class SocketCan {
public:
    SocketCan() = default;
    ~SocketCan();

    // We own a file descriptor, so no copying.
    SocketCan(const SocketCan&) = delete;
    SocketCan& operator=(const SocketCan&) = delete;

    // interface_name is what shows up in `ip link`, ie "can0".
    bool open(const std::string& interface_name, bool enable_can_fd);
    void close();
    bool is_open() const { return socket_fd >= 0; }

    // Non blocking. Returns true when a frame was read into frame_out.
    // When it returns false, no_data_out tells you whether the socket was simply
    // empty (normal) or something actually went wrong (check last_error()).
    bool read_frame(struct canfd_frame* frame_out, bool* no_data_out);

    bool write_frame(const struct canfd_frame& frame);

    const std::string& last_error() const { return last_error_msg; }

private:
    int socket_fd = -1;
    bool can_fd_enabled = false;
    std::string last_error_msg;
};