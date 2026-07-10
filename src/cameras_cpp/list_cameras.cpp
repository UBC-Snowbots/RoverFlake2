// list_cameras.cpp — print libcamera camera indices and IDs.
// Broken Build?:  g++ -std=c++17 list_cameras.cpp -o list_cameras $(pkg-config --cflags --libs libcamera)
//nvm this for build:
// g++ -std=c++17 list_cameras.cpp -o list_cameras -I/opt/ros/humble/include/libcamera -L/opt/ros/humble/lib -lcamera -lcamera-base
// Run:    ./list_cameras

#include <libcamera/libcamera.h>

#include <iostream>
#include <memory>

int main()
{
    libcamera::CameraManager manager;

    int ret = manager.start();
    if (ret) {
        std::cerr << "Failed to start CameraManager (error " << ret << ")\n";
        return ret;
    }

    const auto cameras = manager.cameras();
    if (cameras.empty()) {
        std::cout << "No cameras found.\n";
    } else {
        for (std::size_t i = 0; i < cameras.size(); ++i)
            std::cout << i << ": " << cameras[i]->id() << '\n';
    }

    manager.stop();
    return 0;
}
