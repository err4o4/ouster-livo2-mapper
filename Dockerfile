#FROM dustynv/ros:noetic-desktop-l4t-r35.4.1
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    CATKIN_WS=/opt/catkin_ws

# Refresh ROS GPG key BEFORE update
RUN apt-get install -y --no-install-recommends curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && rm -rf /var/lib/apt/lists/*

# Install pyusb to toggle camera to trigger mode
RUN pip3 install pyusb

# Install dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    git build-essential pkg-config \
    v4l-utils iputils-ping net-tools usbutils \
    nano wget cmake \
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

# Clone and build Sophus
WORKDIR /tmp
RUN git clone https://github.com/strasdat/Sophus.git && cd Sophus && git checkout a621ff2e

# patch + build (run CMake in the Sophus folder)
WORKDIR /tmp/Sophus
RUN sed -i -E \
    -e 's/unit_complex_\.real\(\)\s*=\s*([0-9eE+\.-]+);/unit_complex_.real(\1);/' \
    -e 's/unit_complex_\.imag\(\)\s*=\s*([0-9eE+\.-]+);/unit_complex_.imag(\1);/' \
    sophus/so2.cpp \
 && cmake -S . -B build \
 && cmake --build build -j"$(nproc)" \
 && cmake --install build

# ---------- Create catkin workspace ----------
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

# ---------- Clone repos ----------
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git
RUN git clone -b noetic https://github.com/ros-perception/vision_opencv.git
RUN git clone -b noetic-devel https://github.com/ros-perception/image_transport_plugins.git
RUN git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
RUN git clone --branch 0.3.7 https://github.com/ros-drivers/usb_cam.git
RUN git clone -b noetic https://github.com/ros-perception/image_pipeline.git
RUN git clone https://github.com/err4o4/ros-econ-trigger-camera.git

# ---------- Clone FAST-Calib ----------
RUN git clone https://github.com/err4o4/FAST-LIVO2.git
RUN git clone https://github.com/err4o4/FAST-Calib.git

# ---------- Build (catkin_make) ----------
WORKDIR ${CATKIN_WS}
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release VERBOSE=1 && \
    echo 'source ${CATKIN_WS}/devel/setup.bash' >> /etc/bash.bashrc"

# ---------- Default shell ----------
CMD ["/bin/bash"]