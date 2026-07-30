#include "socket_can.h"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

SocketCan::~SocketCan()
{
    close();
}

bool SocketCan::open(const std::string& interface_name, bool enable_can_fd)
{
    close();

    socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        last_error_msg = std::string("socket() failed: ") + std::strerror(errno);
        return false;
    }

    // Look up the interface index for the name we were given.
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        last_error_msg = "interface '" + interface_name + "' not found: " + std::strerror(errno);
        close();
        return false;
    }

    if (enable_can_fd) {
        int enable = 1;
        if (::setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
            last_error_msg = std::string("could not enable CAN FD frames: ") + std::strerror(errno);
            close();
            return false;
        }
    }
    can_fd_enabled = enable_can_fd;

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        last_error_msg = std::string("bind() failed: ") + std::strerror(errno);
        close();
        return false;
    }

    // Non blocking, so the rx timer can drain whatever is waiting and return
    // instead of blocking the executor.
    int flags = ::fcntl(socket_fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        last_error_msg = std::string("could not set O_NONBLOCK: ") + std::strerror(errno);
        close();
        return false;
    }

    last_error_msg.clear();
    return true;
}

void SocketCan::close()
{
    if (socket_fd >= 0) {
        ::close(socket_fd);
        socket_fd = -1;
    }
}

bool SocketCan::read_frame(struct canfd_frame* frame_out, bool* no_data_out)
{
    *no_data_out = false;
    if (socket_fd < 0) {
        last_error_msg = "socket is not open";
        return false;
    }

    std::memset(frame_out, 0, sizeof(*frame_out));
    ssize_t nbytes = ::recv(socket_fd, frame_out, sizeof(struct canfd_frame), 0);
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            *no_data_out = true;   // nothing waiting, totally normal
            return false;
        }
        last_error_msg = std::string("recv() failed: ") + std::strerror(errno);
        return false;
    }

    // The kernel gives us a whole frame or nothing.
    if (nbytes != (ssize_t)CAN_MTU && nbytes != (ssize_t)CANFD_MTU) {
        last_error_msg = "read an incomplete CAN frame";
        return false;
    }
    return true;
}

bool SocketCan::write_frame(const struct canfd_frame& frame)
{
    if (socket_fd < 0) {
        last_error_msg = "socket is not open";
        return false;
    }

    ssize_t nbytes = 0;
    if (can_fd_enabled) {
        nbytes = ::write(socket_fd, &frame, CANFD_MTU);
        if (nbytes != (ssize_t)CANFD_MTU) {
            last_error_msg = std::string("write() failed: ") + std::strerror(errno);
            return false;
        }
        return true;
    }

    // Classic CAN: the socket expects a can_frame, not a canfd_frame.
    struct can_frame classic;
    std::memset(&classic, 0, sizeof(classic));
    classic.can_id = frame.can_id;
    classic.can_dlc = frame.len;
    std::memcpy(classic.data, frame.data, frame.len);

    nbytes = ::write(socket_fd, &classic, CAN_MTU);
    if (nbytes != (ssize_t)CAN_MTU) {
        // ENOBUFS here almost always means the tx queue is full - the bus is
        // saturated, or nothing is ACKing our frames (no other node powered on).
        last_error_msg = std::string("write() failed: ") + std::strerror(errno);
        return false;
    }
    return true;
}