# Ubuntu runs inside the image on every host. The ROS base image provides both
# linux/amd64 (Fedora and Intel Mac) and linux/arm64 (Apple Silicon).
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROVERFLAKE_ROOT=/RoverFlake2 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=101

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    cmake \
    curl \
    git \
    libgtkmm-3.0-dev \
    libsfml-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-jazzy-desktop \
    ros-jazzy-rmw-cyclonedds-cpp \
    sudo \
    tzdata \
    wget \
    && rm -rf /var/lib/apt/lists/*

# drive_control links against the Phidget C SDK, which its package.xml does
# not declare. Use the vendor repository as the previous setup script did.
RUN wget -qO /usr/share/keyrings/phidgets.gpg \
        https://www.phidgets.com/gpgkey/pubring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/phidgets.gpg] https://www.phidgets.com/debian noble main" \
        > /etc/apt/sources.list.d/phidgets.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends libphidget22-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROVERFLAKE_ROOT

# The old setup script was removed during the Jazzy migration. Install the
# package.xml dependencies with the rosdep command it previously ran.
# gazebo_ros is a stale Gazebo Classic dependency; OpenCV is already installed
# by the desktop package but its capitalized manifest key has no rosdep rule.
COPY src/ $ROVERFLAKE_ROOT/src/
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src \
    --skip-keys="serial moteus_msgs gazebo_ros OpenCV" -y --rosdistro jazzy

COPY . $ROVERFLAKE_ROOT

ENTRYPOINT ["/RoverFlake2/docker/entrypoint.sh"]
CMD ["/bin/bash"]
