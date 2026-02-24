# Build with 'docker compose build'
# Create and enter container with 'docker compose run rover bash'
# if you have anymore questions ask Aaron

# official base image in docs (pinned to jammy for reproducibility)
FROM osrf/ros:humble-desktop-jammy

# consistency with setup files
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV ROVERFLAKE_ROOT=/RoverFlake2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# install base dependencies (recommend not to change if you want to add niche deps)
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    sudo \
    curl \
    git \
    wget \
    bash \
    python3 \
    python3-pip \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# common ROS package deps (use this for niche deps)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-urdf \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-xacro \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROVERFLAKE_ROOT

# copy setup scripts and package manifests first for better layer caching
COPY setup_scripts/ $ROVERFLAKE_ROOT/setup_scripts/
COPY src/ $ROVERFLAKE_ROOT/src/

# run full setup to install dependencies inside the image (auto-confirm prompts)
RUN yes | bash setup_scripts/setup_everything_common.sh

# copy everything else (code changes invalidate from here, but deps are cached)
COPY . $ROVERFLAKE_ROOT

# set entrypoint (already copied above)
RUN chmod +x docker/entrypoint.sh

ENTRYPOINT ["/RoverFlake2/docker/entrypoint.sh"]

CMD ["/bin/bash"]
