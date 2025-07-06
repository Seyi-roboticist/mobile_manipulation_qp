# syntax=docker/dockerfile:1

ARG UBUNTU_VERSION="jammy"
FROM ubuntu:${UBUNTU_VERSION}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

# Install common development tools, locales, rosdep, and vcstool
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    lsb-release \
    python3-pip \
    python3-venv \
    nano \
    vim \
    sudo \
    software-properties-common \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale for UTF-8 support
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Add ROS 2 apt repository using the .deb package method
# First, add the universe repository for add-apt-repository
RUN add-apt-repository universe \
    && apt-get update

# Get the latest ros-apt-source version and install the .deb
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble Desktop
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    # Install colcon tools
    python3-colcon-common-extensions \
    # Install Gazebo ROS bridge if needed
    ros-humble-gazebo-ros-pkgs \
    # Dev tools
    ros-dev-tools \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (run as root before switching user)
# These commands need to be run only once for the system
RUN rosdep init \
    && rosdep update

# Setup ROS 2 environment sourcing for new shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Create a non-root user and add to sudoers
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# Set the working directory
WORKDIR /home/$USERNAME/ros2_ws