FROM koide3/glim_ros2:jazzy_cuda12.5

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -qq && \
    apt-get install -yqq \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-jazzy-autoware-adapi-v1-msgs \
      ros-jazzy-autoware-cmake \
      ros-jazzy-autoware-control-msgs \
      ros-jazzy-autoware-internal-planning-msgs \
      ros-jazzy-autoware-lanelet2-extension \
      ros-jazzy-autoware-lint-common \
      ros-jazzy-autoware-localization-msgs \
      ros-jazzy-autoware-map-msgs \
      ros-jazzy-autoware-perception-msgs \
      ros-jazzy-autoware-planning-msgs \
      ros-jazzy-autoware-system-msgs \
      ros-jazzy-autoware-utils-geometry \
      ros-jazzy-autoware-utils-math \
      ros-jazzy-autoware-utils-rclcpp \
      ros-jazzy-autoware-vehicle-msgs \
      ros-jazzy-geographic-msgs \
      ros-jazzy-lanelet2-io \
      ros-jazzy-pcl-conversions \
      ros-jazzy-rclpy-message-converter \
      ros-jazzy-ros-testing \
      ros-jazzy-diagnostic-updater \
      libgeographiclib-dev \
      libgoogle-glog-dev \
      libpcl-dev \
      librange-v3-dev && \
    rm -rf /var/lib/apt/lists/*
