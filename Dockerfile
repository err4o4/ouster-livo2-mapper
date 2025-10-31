# syntax=docker/dockerfile:1

# For Jetson and arm64 systems
#FROM dustynv/ros:noetic-desktop-l4t-r35.4.1
# For x86 
#FROM osrf/ros:noetic-desktop-full

ARG BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    CATKIN_WS=/opt/catkin_ws

# Refresh ROS GPG key BEFORE update
RUN apt-get install -y --no-install-recommends curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    git build-essential pkg-config \
    v4l-utils iputils-ping net-tools usbutils \
    nano wget cmake python3-dev python3-pip python3-tk \
    libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev \
    libjsoncpp-dev libeigen3-dev libspdlog-dev libcurl4-openssl-dev \
    libpcl-dev libturbojpeg0-dev libjpeg-dev libpng-dev libtiff-dev \
    libtheora-dev zlib1g-dev \
    ros-noetic-rviz ros-noetic-rqt-bag \
    ros-noetic-foxglove-bridge ros-noetic-visualization-msgs \
    ros-noetic-geometry-msgs ros-noetic-nav-msgs \
    ros-noetic-std-msgs ros-noetic-stereo-msgs \
    ros-noetic-tf ros-noetic-tf-conversions \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    ros-noetic-eigen-conversions ros-noetic-camera-info-manager 

# Install pyusb to toggle camera to trigger mode
RUN pip3 install pyusb
RUN pip3 install matplotlib

# patch + build (run CMake in the Sophus folder)
WORKDIR /tmp
RUN git clone https://github.com/strasdat/Sophus.git && cd Sophus && git checkout a621ff2e
WORKDIR /tmp/Sophus
RUN sed -i -E \
    -e 's/unit_complex_\.real\(\)\s*=\s*([0-9eE+\.-]+);/unit_complex_.real(\1);/' \
    -e 's/unit_complex_\.imag\(\)\s*=\s*([0-9eE+\.-]+);/unit_complex_.imag(\1);/' \
    sophus/so2.cpp \
 && cmake -S . -B build \
 && cmake --build build -j"$(nproc)" \
 && cmake --install build

# Install Livox SDK
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /tmp/Livox-SDK
RUN cd build && cmake .. && make -j"$(nproc)" && make install

# Install Ceres
WORKDIR /tmp
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.0.0.tar.gz && tar zxf 2.0.0.tar.gz
WORKDIR /tmp/ceres-solver-2.0.0 
RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ../../ceres-solver-2.0.0 && make -j"$(nproc)" && make install 

# ---------- Create catkin workspace ----------
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

# ---------- Clone repos ----------
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git
RUN git clone -b noetic https://github.com/ros-perception/vision_opencv.git
RUN git clone -b noetic-devel https://github.com/ros-perception/image_transport_plugins.git

RUN git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

RUN git clone --branch 0.3.7 https://github.com/ros-drivers/usb_cam.git
RUN git clone https://github.com/err4o4/ros-econ-trigger-camera.git
RUN git clone -b noetic https://github.com/ros-perception/image_pipeline.git
RUN git clone https://github.com/err4o4/ros-supervisor

# ---------- Clone FAST-* ----------
RUN git clone https://github.com/err4o4/FAST-Calib.git
RUN git clone https://github.com/err4o4/FAST-LIVO2.git

# ---------- Build (catkin_make) ----------
WORKDIR ${CATKIN_WS}
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

WORKDIR ${CATKIN_WS}/src
RUN git clone https://github.com/err4o4/LiDAR_IMU_Init.git
RUN git clone https://github.com/hku-mars/FAST_LIO.git && \
    cd FAST_LIO && \
    git submodule update --init

WORKDIR ${CATKIN_WS}
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg fast_lio lidar_imu_init"

# Setup ROS environment in bashrc (separate step, idempotent)
RUN grep -qxF "source ${CATKIN_WS}/devel/setup.bash" /etc/bash.bashrc || \
    echo "source ${CATKIN_WS}/devel/setup.bash" >> /etc/bash.bashrc

# ---------- Copy and setup startup script ----------
COPY /startup.sh /root/startup.sh
RUN chmod +x /root/startup.sh

# ---------- Default command ----------
CMD ["/root/startup.sh"]