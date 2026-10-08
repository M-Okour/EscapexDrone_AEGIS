# Development image for laptops (x86_64 or ARM Mac via emulation).
#   docker build -t exd -f docker/Dockerfile .
#   docker run -it --rm -v $(pwd):/ws exd
# Inside:  colcon build --symlink-install && source install/setup.bash
#
# On the Jetson use the native ROS 2 Humble install on JetPack 6 (Ubuntu 22.04),
# or a dustynv/ros:humble-* container — see docs/JETSON.md.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pip python3-opencv \
      ros-humble-rviz2 ros-humble-rqt-robot-monitor ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /tmp/src
RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/* /tmp/src

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
