FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release software-properties-common sudo \
    build-essential git \
    python3-pip \
    libceres-dev libeigen3-dev libtbb-dev libomp-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    libusb-1.0-0-dev \
    libboost-all-dev \
    libmetis-dev \
    libfmt-dev \
    libspdlog-dev \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    ca-certificates \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    libosmesa6 \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir colcon-common-extensions

RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && \
    git checkout 4.3a0 && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/gtsam

RUN git clone https://github.com/koide3/iridescence --recursive /tmp/iridescence && \
    mkdir /tmp/iridescence/build && \
    cd /tmp/iridescence/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/iridescence

RUN git clone https://github.com/koide3/gtsam_points /tmp/gtsam_points && \
    mkdir /tmp/gtsam_points/build && \
    cd /tmp/gtsam_points/build && \
    cmake .. -DBUILD_WITH_CUDA=OFF && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/gtsam_points

WORKDIR /ros2_ws

COPY ./src ./src

RUN ldconfig

RUN sed -i \
  -e 's|"imu_topic": "/os_cloud_node/imu",|"imu_topic": "/livox/imu",|' \
  -e 's|"points_topic": "/os_cloud_node/points",|"points_topic": "/livox/pointcloud" ,|' \
  -e 's|"acc_scale": 1.0,|"acc_scale": 9.80665,|' \
  src/glim/config/config_ros.json

RUN sed -i \
  -e 's|0.006, -0.012, 0.008, 0.0, 0.0, 0.0, 1.0|-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0|' \
  src/glim/config/config_sensors.json

RUN sed -i \
  -e 's|"config_odometry": "config_odometry_gpu.json"|"config_odometry": "config_odometry_cpu.json"|' \
  -e 's|"config_sub_mapping": "config_sub_mapping_gpu.json"|"config_sub_mapping": "config_sub_mapping_cpu.json"|' \
  -e 's|"config_global_mapping": "config_global_mapping_gpu.json"|"config_global_mapping": "config_global_mapping_cpu.json"|' \
  src/glim/config/config.json

RUN source /opt/ros/humble/setup.bash && \
    colcon build

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros

RUN python3 -m pip install "rosbags==0.10.5"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
