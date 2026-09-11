# Adds only the standard ROS image decoder needed by the RTK-SLAM release,
# whose hardware-triggered camera stream is stored as CompressedImage.
# FAST-LIVO2 itself remains the clean, profile-pinned checkout mounted at
# /bench/FAST-LIVO2 by the benchmark runner.
FROM fast-livo2-benchmark:noetic

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-noetic-compressed-image-transport \
 && rm -rf /var/lib/apt/lists/*

LABEL benchmark.fast_livo2.camera_transport="ros-noetic-compressed-image-transport"
