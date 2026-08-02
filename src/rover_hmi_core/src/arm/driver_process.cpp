#include "driver_process.h"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <glob.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <string>

bool DriverProcess::localAdapterPresent() {
    if (::access("/dev/fdcanusb", F_OK) == 0) return true;
    glob_t g{};
    const bool found =
        ::glob("/dev/serial/by-id/*fdcanusb*", 0, nullptr, &g) == 0 && g.gl_pathc > 0;
    globfree(&g);
    return found;
}

bool DriverProcess::running() {
    if (pid_ <= 0) return false;
    int status;
    if (::waitpid(pid_, &status, WNOHANG) == pid_) { pid_ = -1; return false; }
    return true;
}

bool DriverProcess::start() {
    if (running()) return true;

    std::string bin;
    try {
        bin = ament_index_cpp::get_package_prefix("arm_hardware_interface")
            + "/lib/arm_hardware_interface/moteus_driver";
    } catch (...) { return false; }

    const pid_t p = ::fork();
    if (p < 0) return false;
    if (p == 0) {
        ::prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (::getppid() == 1) ::_exit(1);   // HMI died between fork and prctl
        ::execl(bin.c_str(), bin.c_str(), (char*)nullptr);
        ::_exit(127);
    }
    pid_ = p;
    return true;
}

void DriverProcess::stop() {
    if (pid_ <= 0) return;
    ::kill(pid_, SIGINT);
    for (int i = 0; i < 10; i++) {          // ~1 s grace for a clean rclcpp shutdown
        int status;
        if (::waitpid(pid_, &status, WNOHANG) == pid_) { pid_ = -1; return; }
        ::usleep(100000);
    }
    ::kill(pid_, SIGKILL);
    int status;
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
}
