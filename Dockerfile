ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update
RUN apt-get install -y nlohmann-json3-dev libpcl-dev python3-pip
RUN pip3 install rosbags --break-system-packages
RUN mkdir -p /test_ws/src
COPY src/ /test_ws/src
# Skip building glim and glim_ros2 - we only need the glim-to-hdmapping converter
RUN touch /test_ws/src/glim/COLCON_IGNORE /test_ws/src/glim_ros2/COLCON_IGNORE
RUN cd /test_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y || true && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build
