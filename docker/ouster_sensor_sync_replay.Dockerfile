# ROS1 decoder for the packet-only NTNU LiDAR Degeneracy Datasets.
# The dataset authors require this sensor-sync branch so that each Ouster
# scan is stamped from /sensor_sync_node/trigger_2 rather than the sensor's
# short, wrapping packet clock.
FROM fast-livo2-benchmark:noetic

ARG OUSTER_ROS_REVISION=e6531d006e008942cc15c8334f8257dc3e4855d2

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libcurl4-openssl-dev \
      libeigen3-dev \
      libjsoncpp-dev \
      libspdlog-dev \
      ros-noetic-nodelet \
      ros-noetic-pcl-ros \
 && rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules \
      https://github.com/ntnu-arl/ouster-ros.git \
      /opt/ouster_sync_ws/src/ouster_ros \
 && git -C /opt/ouster_sync_ws/src/ouster_ros checkout "${OUSTER_ROS_REVISION}" \
 && git -C /opt/ouster_sync_ws/src/ouster_ros submodule update --init --recursive \
 && . /opt/ros/noetic/setup.sh \
 && cd /opt/ouster_sync_ws \
 && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

LABEL benchmark.ouster_ros.revision="e6531d006e008942cc15c8334f8257dc3e4855d2" \
      benchmark.ouster_ros.branch="dev/sensor_sync_replay" \
      benchmark.dataset="ntnu-arl/lidar_degeneracy_datasets"

