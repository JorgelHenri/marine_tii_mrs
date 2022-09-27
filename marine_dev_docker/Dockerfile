
# Inspired from vrx and Davesims dockers
# FROM ros:noetic
# FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04
# FROM nvidia/cuda:11.0-base
FROM nvidia/cuda:11.4.1-devel-ubuntu20.04

# If you don't setup your timezone there will be a prompt during installation
# and it will crash
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata

# This is to change default user password

ARG USER_NAME="tii"
ARG USER_PASSWORD="tii"

# And make it accessible via RUN commands

ENV USER_NAME $USER_NAME
ENV USER_PASSWORD $USER_PASSWORD

# Set Ubuntu release
ARG RELEASE=focal

# Set ROS distribution
ARG DIST=noetic

# Set Gazebo version
ARG GAZ=gazebo11

RUN apt update \
    && apt install -y git zsh wget \
        build-essential \
        cmake \
        cppcheck \
        curl \
        git \
        gnupg \
        libeigen3-dev \
        libgles2-mesa-dev \
        lsb-release \
        pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-pip \
        python3-venv \
        qtbase5-dev \
        ruby \
        software-properties-common \
        sudo \
        vim \
        gdb \
        libbluetooth-dev \
        libcwiid-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        # python2-dbg \
        # python2-empy \
        # python2-numpy \
        # python2-setuptools \
        # python2-pip \
        net-tools \
        iputils-ping \
        libyaml-cpp-dev \
        htop \
        bash-completion \
    && apt clean

# Get ROS and Gazebo
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${RELEASE} main" > /etc/apt/sources.list.d/ros1-latest.list' \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable ${RELEASE} main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
    && apt update \
    && apt install -y --no-install-recommends \
        ${GAZ} \
        lib${GAZ}-dev \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-vcstool \
        ros-${DIST}-gazebo-plugins \
        ros-${DIST}-gazebo-ros \
        ros-${DIST}-gazebo-ros-control \
        ros-${DIST}-gazebo-ros-pkgs \
        ros-${DIST}-effort-controllers \
        ros-${DIST}-geographic-info \
        ros-${DIST}-hector-gazebo-plugins \
        ros-${DIST}-joint-state-controller \
        ros-${DIST}-joint-state-publisher \
        ros-${DIST}-joy \
        ros-${DIST}-joy-teleop \
        ros-${DIST}-kdl-parser-py \
        ros-${DIST}-key-teleop \
        ros-${DIST}-move-base \
        ros-${DIST}-robot-localization \
        ros-${DIST}-robot-state-publisher \
        ros-${DIST}-ros-base \
        ros-${DIST}-rqt \
        ros-${DIST}-rqt-common-plugins \
        ros-${DIST}-rqt-robot-plugins \
        ros-${DIST}-rviz \
        ros-${DIST}-teleop-tools \
        ros-${DIST}-teleop-twist-joy \
        ros-${DIST}-teleop-twist-keyboard \
        ros-${DIST}-tf2-geometry-msgs \
        ros-${DIST}-tf2-tools \
        ros-${DIST}-urdfdom-py \
        ros-${DIST}-velodyne-gazebo-plugins \
        ros-${DIST}-velodyne-simulator \
        ros-${DIST}-xacro \
    && apt clean

# Nukhada navigation dependencies
# RUN sudo apt-get update && sudo apt-get install -y \
#         ros-${DIST}-pointcloud-to-laserscan \
#         ros-${DIST}-geodesy \
#         ros-${DIST}-pcl-ros \
#         ros-${DIST}-nmea-msgs \
#         ros-${DIST}-libg2o \
#         ros-${DIST}-velodyne-description \
#         ros-${DIST}-pcl-msgs \
#         ros-${DIST}-perception \
#         ros-${DIST}-perception-pcl \
#         ros-${DIST}-pluginlib \
#         ros-${DIST}-gmapping \
#         ros-${DIST}-costmap-converter \
#         ros-${DIST}-teb-local-planner \
#         ros-${DIST}-mpc-local-planner \
#         ros-${DIST}-tf2-eigen \
#         ros-${DIST}-global-planner \
#         ros-${DIST}-dwa-local-planner \
#         libglfw3 \
#         libglfw3-dev \
#         libpopt-dev \
#     && apt clean 

# Optional: Dev. tools, applications, etc...
# GUI rendering
# for a production environment you'll want to remove most of this RUN line (GUI stuff will
# anyway probably not even build since target is ARM)
# TODO: make the i386 conditionnal on the target platorm
# or run this line conditionnaly on a parameter given in command line
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    vim \
    python3-vcstool && \
    dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
    libxau6 libxau6:i386 \
    libxdmcp6 libxdmcp6:i386 \
    libxcb1 libxcb1:i386 \
    libxext6 libxext6:i386 \
    libx11-6 libx11-6:i386 \
    libglvnd0 libglvnd0:i386 \
    libgl1 libgl1:i386 \
    libglx0 libglx0:i386 \
    libegl1 libegl1:i386 \
    libgles2 libgles2:i386 && \
    echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

COPY NGC-DL-CONTAINER-LICENSE /
COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

##
# Add your dependencies here !
# RUN apt-get update && apt-get install -y --no-install-recommends ...
#
##

RUN apt update && apt install -y tree

RUN apt install -y ros-noetic-joint-state-publisher-gui

# asv_wave_sim
RUN apt install -y libcgal-dev fftw-dev libfftw3-dev libclfft-dev python3-catkin-tools

# python_coverage_planning
RUN apt install -y libgmp-dev libmpfr-dev
RUN apt install -y mono-devel
RUN apt install -y libgoogle-glog-dev

RUN pip3 install scipy

RUN pip install pymap3d

RUN pip install casadi

RUN pip install ruamel.yaml

RUN sudo apt install -y texlive-xetex texlive-fonts-recommended texlive-plain-generic

RUN sudo apt install -y jupyter-nbconvert

# For fixing Cola2 calling /usr/bin/env python by default while on Ubuntu20.04
# we have python3 by default
# RUN sudo apt install -y python-is-python3

# Create the user

RUN adduser --quiet --disabled-password --shell /bin/zsh --home /home/$USER_NAME $USER_NAME

# Change its password and add him to sudo group
RUN echo "${USER_NAME}:${USER_PASSWORD}" | chpasswd && usermod -aG sudo $USER_NAME

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

# Put the entrypoint script in the containers FS anyway
COPY ./entrypoint.sh /

RUN chmod +x /entrypoint.sh

# Log as this user
USER $USER_NAME

# Cola2 lib
# installed from /home/tii/dev because
# workspace/src symlink can't be followed from
# inside the Dockerfile
RUN mkdir /home/tii/dev && cd /home/tii/dev && \
    git clone https://bitbucket.org/iquarobotics/cola2_lib.git && \
    cd cola2_lib && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    sudo make install


# Terminal colors if I recall
ENV TERM xterm

# My favorite oh-my-zsh theme
ENV ZSH_THEME agnoster

# Install oh-my-zsh
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

# VSCode docker plugin somehow overwrites the Entrypoint instruction, so source here

RUN echo "source /opt/ros/noetic/setup.zsh" >> /home/tii/.zshrc

# VCS autocompletion
RUN echo "source /usr/share/vcstool-completion/vcs.zsh" >> /home/tii/.zshrc

RUN echo "source /home/tii/workspace/devel/setup.zsh" >> /home/tii/.zshrc

RUN echo "export LANG=\"C.UTF-8\"" >> /home/tii/.zshrc

# If you want to include a pre-cloned / pre-built
# ros package you have to temporarily copy your ssh keys
# to give access to Docker at the build stage
# Don't forget to remove them and avoid this in general
# Following is left as an example
#COPY .ssh /home/tii/.ssh
#USER root
#RUN sudo chmod 777 /home/tii/.ssh/*
# Log as this user
#USER $USER_NAME
#RUN cd /home/tii && git clone --branch scaled-usv git@bitbucket.org:autonomouscv/ros_management_example.git
#RUN . /opt/ros/noetic/setup.sh && cd /home/tii && mkdir -p tii_ws/src && cd tii_ws && ls && vcs import --input /home/tii/ros_management_example/tii.yml --recursive --skip-existing --debug --repos  src
#RUN . /opt/ros/noetic/setup.sh && cd /home/tii/tii_ws ls && catkin build
#RUN . /opt/ros/noetic/setup.sh && cd /home/tii/tii_ws/src/asv_wave_sim/asv_wave_sim && catkin clean --this && catkin build --this --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
#USER root
#RUN rm -rf /home/tii/.ssh
#USER $USER_NAME

# ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "zsh" ]
