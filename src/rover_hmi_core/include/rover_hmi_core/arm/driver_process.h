// driver_process.h — bench-only local moteus_driver lifecycle for the HMI.
//
// Spawns the driver binary directly (not via `ros2 run`, whose Python wrapper
// would absorb the death signal and orphan the real process) with
// PR_SET_PDEATHSIG, so the driver dies with the HMI even on a crash-close.
// On the rover the driver is started by bringup on the NUC instead; the
// Start button only enables when an fdcanusb is present on this machine.

#pragma once

#include <sys/types.h>

class DriverProcess {
public:
    static bool localAdapterPresent();   // /dev/fdcanusb or by-id match on this host

    bool running();                      // child alive? (reaps if it exited)
    bool start();                        // fork + exec moteus_driver, PDEATHSIG-tied
    void stop();                         // SIGINT, brief grace, then SIGKILL

private:
    pid_t pid_ = -1;
};
