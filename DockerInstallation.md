# Docker setup for RoverFlake2

Install Docker with the Compose plugin. The container uses Ubuntu 24.04 and ROS
Jazzy on every host; Fedora and macOS do not need ROS installed on the host.
Clone the repository with its submodules before building:

```sh
git submodule update --init --recursive
```

## Fedora

The Fedora Compose file uses the host network, `/dev`, and the X11 socket for
rover hardware and GUI access. If your desktop uses Wayland, Xwayland must be
running and `DISPLAY` must be set. Permit local root access to X11 before
starting a GUI application:

```sh
xhost +si:localuser:root
docker compose -f docker-compose.fedora.yml up -d --build rover
docker compose -f docker-compose.fedora.yml exec rover bash
```

## macOS (Intel or Apple Silicon)

Docker Desktop runs this Linux image as amd64 on Intel Macs and arm64 on Apple
Silicon. Start the container with:

```sh
docker compose -f docker-compose.macos.yml up -d --build rover
docker compose -f docker-compose.macos.yml exec rover bash
```

For Linux GUI windows on macOS, install and start XQuartz, enable **Allow
connections from network clients** in its settings, restart XQuartz, then run
`xhost +localhost` on the Mac before starting the container. The Compose file
routes `DISPLAY` through `host.docker.internal`. Docker Desktop does not pass
through the host's USB or CAN devices, so hardware nodes need a Linux host.

The container's entrypoint sources ROS Jazzy and attempts a workspace build
when no `install/setup.bash` exists. Its source tree is bind mounted from this
checkout, so local edits are visible immediately inside the container.
