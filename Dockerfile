# Build with 'docker compose build'
# Create and enter container with 'docker compose run rover bash'
# if you have anymore questions ask Aaron

# official base image in docs
FROM osrf/ros:humble-desktop

# consistency with setup files
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV ROVERFLAKE_ROOT=/RoverFlake2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# install base depenencies (recommend not to change if you want to add niche deps)
RUN apt-get update && apt-get install -y \
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

# copy root into container
WORKDIR $ROVERFLAKE_ROOT
COPY . $ROVERFLAKE_ROOT

# run the full setup script (confirms everything including nested setup scripts)
RUN yes | bash setup_scripts/setup_everything_common.sh

# copy and set entrypoint (runs setup)
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# changed with COPY
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]
