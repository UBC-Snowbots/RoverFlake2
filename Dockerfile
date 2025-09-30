# Run with 'docker build -t roverflake2:<container_name>'
# Enter container with 'docker run -it roverflake2:<container_name>'
# refer to docs if you need more help (or ask Aaron)

FROM ubuntu:22.04

# consistency with setup files
ENV DEBIAN_FRONTEND=noninteractive
ENV ROVERFLAKE_ROOT=/RoverFlake2

# Install base depenencies
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
    && rm -rf /var/lib/apt/lists/*

# copy root into container
WORKDIR $ROVERFLAKE_ROOT
COPY . $ROVERFLAKE_ROOT

# run setup
RUN bash setup_scripts/setup_everything_common.sh

# change profile if you don't like bash
CMD ["/bin/bash"]
