FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y upgrade && apt-get -y install \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    lsb-release \
    gnupg \
    gnupg2 \
    locales \
    net-tools \
    iputils-ping \
    netcat \
    software-properties-common \
    python3-dev \
    python3-pip \
    python-is-python3 && \
    add-apt-repository universe \
    && apt-get -y autoremove \
    && apt-get clean

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV ROS_DISTRO humble
ENV QT_X11_NO_MITSHM 1
ENV TERM xterm-256color
ENV HOME /root
ENV COLCON_WS /root/colcon_ws
ENV COLCON_SRC /root/colcon_ws/src

RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get -y upgrade

RUN apt-get update && apt-get -y upgrade && apt-get -y install \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    && apt-get -y autoremove \
    && apt-get clean

RUN apt-get update && apt-get -y upgrade && apt-get -y install \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && apt-get -y autoremove \
    && apt-get clean

RUN mkdir -p ${COLCON_SRC} && \
    cd ${COLCON_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    cd ${COLCON_WS} && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src && \
    colcon build --symlink-install
