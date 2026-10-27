# syntax=docker/dockerfile:1.7
# CAC 2027 dev environment: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
#   base   ROS 2 + Nav2 + robot_localization + ros2_control + PCL + OpenCV + Foxglove
#   sim    + Gazebo Harmonic (ros_gz) + RViz
#   dev    + debug/test tools, runs as a user with your UID   <- the one you use
#   robot  base only, no GUI, for the rover's computer
FROM ros:jazzy-ros-base-noble AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
# Retry dropped downloads instead of failing a 15-minute build on flaky Wi-Fi.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl ca-certificates \
      python3-colcon-common-extensions python3-rosdep python3-pip \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
      ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-robot-localization \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-pcl-ros ros-jazzy-sensor-msgs-py \
      ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard ros-jazzy-twist-mux \
      ros-jazzy-foxglove-bridge ros-jazzy-rosbag2-storage-mcap \
      python3-opencv python3-numpy python3-scipy python3-yaml \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS in every shell: interactive (bash.bashrc) and login (profile.d).
# `docker compose exec` skips the entrypoint, so without this `ros2` is "not found".
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' \
      | tee /etc/profile.d/10-cac-ros.sh >> /etc/bash.bashrc

FROM base AS sim
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-rviz2 mesa-utils \
    && rm -rf /var/lib/apt/lists/*
ENV QT_X11_NO_MITSHM=1

FROM sim AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb python3-pytest ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
      ros-jazzy-ament-flake8 ros-jazzy-rqt-common-plugins ros-jazzy-plotjuggler-ros \
      vim nano less tmux sudo \
    && rm -rf /var/lib/apt/lists/*
# A user whose UID/GID match yours, so build/ and install/ in your clone aren't owned by root.
# The noble base image already has user 'ubuntu' at 1000:1000; reuse/rename it if UIDs clash.
ARG UID=1000
ARG GID=1000
RUN if getent group ${GID} >/dev/null; then groupmod -n cac "$(getent group ${GID} | cut -d: -f1)"; \
    else groupadd -g ${GID} cac; fi && \
    if getent passwd ${UID} >/dev/null; then \
      usermod -l cac -d /home/cac -m -g ${GID} "$(getent passwd ${UID} | cut -d: -f1)"; \
    else useradd -m -u ${UID} -g ${GID} -s /bin/bash cac; fi && \
    usermod -aG dialout,video cac && \
    echo 'cac ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/cac && \
    mkdir -p /workspace && chown ${UID}:${GID} /workspace
USER cac
WORKDIR /workspace
RUN rosdep update --rosdistro jazzy

FROM base AS robot
WORKDIR /workspace
