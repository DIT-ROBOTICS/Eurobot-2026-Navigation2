##############################################################################
# Base Image: ros2-humble
##############################################################################

FROM osrf/ros:humble-desktop-full-jammy

##############################################################################
# Arguments
##############################################################################

ARG USERNAME=user
ARG USERID=1003
ARG USERGID=1004
ARG SHELL=bash

##############################################################################
# Label
##############################################################################

LABEL org.opencontainers.image.authors="Tu"
LABEL shell="${SHELL}"

##############################################################################
# Set environment variables
##############################################################################

ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}
ENV TERM=xterm-256color
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble

##############################################################################
# Install packages
##############################################################################

RUN apt-get update && \
    apt-get install -y \
    sudo \
    vim \
    curl \
    wget \
    tmux \
    htop \
    git \
    git-extras \
    gnupg2 \
    net-tools \
    locales \
    build-essential \
    cmake \
    python3-colcon-common-extensions

# Install dependencies for ROS and Nav2
RUN apt-get update && \
    apt-get install -y \
    libsdl-image1.2-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libyaml-cpp-dev \
    ros-humble-tf2-sensor-msgs \
    ros-humble-libg2o \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-geographic-msgs \
    ros-humble-geometry-msgs \
    ros-humble-test-msgs \
    ros-humble-bondcpp \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp

##############################################################################
# Set Language
##############################################################################

RUN echo "en_US.UTF-8 UTF-8" > /etc/local.gen && locale-gen en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
##############################################################################
# Set timezone (Taipei)
##############################################################################

RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

##############################################################################
# Add new user (Passwordless with sudo)
##############################################################################

RUN groupadd --gid ${USERGID} ${USERNAME} \
    && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
    # passwordless
    && apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
# Set Default User & Working Directory & Default .bashrc
##############################################################################

USER ${USERNAME}
WORKDIR /home/${USERNAME}
COPY ./settings/.bashrc /home/${USERNAME}/.bashrc 

##############################################################################
# Copy tf2_sensor_msgs to avoid error
##############################################################################

COPY ./settings/tf2_sensor_msgs.hpp /opt/ros/humble/include/tf2_sensor_msgs/tf2_sensor_msgs.hpp

##############################################################################
# Clean apt-cache
##############################################################################

RUN sudo apt autoremove -y && \
    sudo apt clean -y 

##############################################################################
# Default command
##############################################################################

SHELL ["/bin/bash", "-c"]
CMD ["bash"]
