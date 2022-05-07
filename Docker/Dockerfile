FROM arm64v8/ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV ROS2_DISTRO foxy

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=argnctu \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER}

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

###################################### basic tools #####################################

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    make \
    git \
    vim \
    gedit \
    wget \
    curl \
    sudo \
    lsb-release \
    build-essential \
    net-tools \
    dirmngr \
    gnupg2 \
    mercurial \
    iputils-ping \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libpcap-dev \
    libpython3-dev \
    libhidapi-dev \
    libopencv-dev \
    libopencv-viz-dev \
    libhidapi-libusb0 \
    python-gobject \
    python-tk \
    python3-pip \
    python3-setuptools \
    python3-opencv \
    python3-numpy \
    python3-empy  \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

###################################### ROS #####################################

# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# # install bootstrap tools
# RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
#     ros-$ROS_DISTRO-desktop-full \
#     ros-$ROS_DISTRO-ddynamic-reconfigure \
#     ros-$ROS_DISTRO-tf2* \
#     ros-$ROS_DISTRO-teleop-twist-joy* \
#     ros-$ROS_DISTRO-rosserial \
#     ros-$ROS_DISTRO-rosserial-arduino \
#     python3-rosdep \
#     python3-rosinstall \
#     python3-rosinstall-generator \
#     python3-wstool \
#     && rm -rf /var/lib/apt/lists/*

###################################### ROS2 #####################################

# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
#     ros-$ROS2_DISTRO-desktop \
#     ros-$ROS2_DISTRO-ros1-bridge \
#     python3-colcon-common-extensions

# RUN rosdep init \
#     && rosdep update

######################################## realsense ########################################

# RUN apt-get update && apt-get install -y \
#     libsuitesparse-dev\
#     software-properties-common \
#     libusb-1.0-0-dev \
#     libx11-dev \
#     xorg-dev \
#     libgl1-mesa-dev \
#     libglu1-mesa-dev \
#     && rm -rf /var/lib/apt/lists/*

# # set the version of the realsense library
# ENV LIBREALSENSE_VERSION 2.36.0
# RUN wget https://github.com/IntelRealSense/librealsense/archive/v${LIBREALSENSE_VERSION}.tar.gz && \
#     tar -xvzf v${LIBREALSENSE_VERSION}.tar.gz && \
#     rm v${LIBREALSENSE_VERSION}.tar.gz && \
#     mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
#     cd librealsense-${LIBREALSENSE_VERSION}/build && \
#     cmake .. && \
#     make && \
#     make install && \
#     rm -rf librealsense-${LIBREALSENSE_VERSION}

##################################### PIP ######################################

RUN pip3 install --upgrade pip setuptools

RUN pip3 install -U \
    rospkg \
    catkin-tools \
    matplotlib \
    pandas \
    gdown \
    argcomplete

##################################### PYNQ ######################################
 
RUN git clone --recursive https://github.com/Xilinx/Kria-PYNQ.git

ENV ARCH=aarch64 \
    BOARD=KV260 \
    PYNQ_ENV=/usr/local/share/pynq-env

# Install Required Debian Packages
RUN apt-key adv --recv-keys --keyserver hkp://keyserver.ubuntu.com:80 \
	        --verbose 803DDF595EA7B6644F9B96B752150A179A9E84C9

RUN echo "deb http://ppa.launchpad.net/ubuntu-xilinx/updates/ubuntu focal main" > /etc/apt/sources.list.d/xilinx-gstreamer.list

RUN apt-get -o DPkg::Lock::Timeout=10 update && apt-get install -y \
    python3.8-venv \
    python3-cffi \
    libssl-dev \
    libcurl4-openssl-dev \
    portaudio19-dev \
    libcairo2-dev \
    libdrm-xlnx-dev \
    libopencv-dev \
    python3-opencv \
    graphviz \
    i2c-tools \
    fswebcam \
    device-tree-compiler \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp && \
    cp -r Kria-PYNQ/pynq/sdbuild/packages/libsds/libcma /tmp && \
    pushd /tmp/libcma && \
    make -t && \
    make install && \
    popd && \
    rm -rf /tmp/libcma

# PYNQ Python Package
RUN python3 -m pip install pynq

# # Get PYNQ Binaries (ublaze compiler and xclbinutils
# RUN pushd /tmp && \
#     wget https://bit.ly/pynq_binaries_2_7 -O pynq_binaries.tar.gz && \
#     tar -xf pynq_binaries.tar.gz && \
#     cp -r /tmp/pynq-v2.7-binaries/gcc-mb/microblazeel-xilinx-elf /usr/local/share/pynq-env/bin/ && \
#     echo "export PATH=\$PATH:/usr/local/share/pynq-venv/bin/microblazeel-xilinx-elf/bin/" >> /etc/profile.d/pynq_env.sh && \
#     cp pynq-v2.7-binaries/xrt/xclbinutil /usr/local/share/pynq-env/bin/ && \
#     chmod +x /usr/local/share/pynq-venv/bin/xclbinutil && \
#     popd

# # define the name of the platform
# RUN echo "${BOARD}" > /etc/xocl.txt

# # Compile pynq device tree overlay and insert it by default
# RUN pushd Kria-PYNQ/dts/ && \
#     make && \
#     mkdir -p /usr/local/share/pynq-env/pynq-dts/ && \
#     cp insert_dtbo.py pynq.dtbo /usr/local/share/pynq-env/pynq-dts/ && \
#     echo "python3 /usr/local/share/pynq-venv/pynq-dts/insert_dtbo.py" >> /etc/profile.d/pynq_venv.sh && \
#     source /etc/profile.d/pynq_venv.sh && \
#     popd

# # Set up pynq pl_server
# RUN cp Kria-PYNQ/pynq/sdbuild/packages/pynq/pl_server.sh /usr/local/bin && \
#     cp Kria-PYNQ/pynq/sdbuild/packages/pynq/pl_server.service /lib/systemd/system && \
#     systemctl enable pl_server

# # Install DPU-PYNQ
# RUN python3 -m pip install pynq-dpu --no-use-pep517

##################################### setting ###################################################
RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers

# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR ${HOME}
