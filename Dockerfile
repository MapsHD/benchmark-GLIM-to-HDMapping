ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

# --- workaround for 403 Forbidden on some Canonical mirrors -----------
RUN printf 'Acquire::http::Pipeline-Depth "0";\nAcquire::http::No-Cache "true";\nAcquire::Retries "3";\n' \
        > /etc/apt/apt.conf.d/99no-pipeline

# Install dependencies
RUN apt-get update && apt-get install -y \
    nlohmann-json3-dev \
    libpcl-dev \
    python3-pip \
    wget nano build-essential clang lld libomp-dev \
    git cmake libeigen3-dev ca-certificates \
    libtbb-dev libboost-all-dev libgtest-dev libmetis-dev \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Build GTSAM from source (required by glim)
ARG GTSAM_TAG=4.3a0
WORKDIR /root
RUN git clone https://github.com/borglab/gtsam.git \
    && cd gtsam \
    && git checkout ${GTSAM_TAG} \
    && mkdir build && cd build \
    && cmake .. \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_USE_SYSTEM_METIS=ON \
    && make -j$(nproc) \
    && make install \
    && rm -rf /root/gtsam


## Build gtsam_points
RUN git clone https://github.com/koide3/gtsam_points && cd gtsam_points && mkdir build && cd build &&cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && rm -rf /root/gtsam_points

RUN pip3 install rosbags --break-system-packages
RUN mkdir -p /test_ws/src
COPY src/ /test_ws/src
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_CUDA=OFF
