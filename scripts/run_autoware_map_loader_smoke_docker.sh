#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "Usage: $0 <autoware_map_dir> [autoware_core_dir] [work_dir]" >&2
  exit 1
fi

MAP_DIR=$(realpath "$1")
AUTOWARE_CORE_DIR=$(realpath "${2:-/tmp/autoware_core}")
WORK_DIR=$(realpath -m "${3:-/tmp/autoware_map_runtime_ws}")
IMAGE=${AUTOWARE_DOCKER_IMAGE:-koide3/glim_ros2:jazzy_cuda12.5}

if [[ ! -d "$MAP_DIR/pointcloud_map" ]]; then
  echo "pointcloud_map directory not found under $MAP_DIR" >&2
  exit 1
fi
if [[ ! -f "$MAP_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
  echo "pointcloud_map_metadata.yaml not found under $MAP_DIR/pointcloud_map" >&2
  exit 1
fi
if [[ ! -f "$MAP_DIR/map_projector_info.yaml" ]]; then
  echo "map_projector_info.yaml not found under $MAP_DIR" >&2
  exit 1
fi
if [[ ! -d "$AUTOWARE_CORE_DIR" ]]; then
  echo "autoware_core directory not found: $AUTOWARE_CORE_DIR" >&2
  exit 1
fi

mkdir -p "$WORK_DIR"
RUN_DIR=$(mktemp -d "$WORK_DIR/run.XXXXXX")

docker run --rm -i \
  -v "$AUTOWARE_CORE_DIR:/autoware_core:ro" \
  -v "$MAP_DIR:/maps:ro" \
  -v "$RUN_DIR:/autoware_ws" \
  "$IMAGE" bash -s -- <<'EOF'
set -eo pipefail

print_log_tail() {
  local path="$1"
  if [[ -f "$path" ]]; then
    echo "==> $path"
    tail -n 40 "$path" || true
  fi
}

P1=""
P2=""
finish() {
  local status=$?
  if [[ -n "$P1" ]]; then
    kill "$P1" 2>/dev/null || true
    wait "$P1" 2>/dev/null || true
  fi
  if [[ -n "$P2" ]]; then
    kill "$P2" 2>/dev/null || true
    wait "$P2" 2>/dev/null || true
  fi
  if [[ $status -ne 0 ]]; then
    print_log_tail /tmp/apt_update.log
    print_log_tail /tmp/apt_install.log
    print_log_tail /tmp/build.log
    print_log_tail /tmp/map_projection_loader.log
    print_log_tail /tmp/pointcloud_map_loader.log
  fi
  exit $status
}
trap finish EXIT

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/tmp/apt_update.log 2>&1
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
  librange-v3-dev >/tmp/apt_install.log 2>&1

set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build \
  --merge-install \
  --build-base /autoware_ws/build \
  --install-base /autoware_ws/install \
  --base-paths \
    /autoware_core/common/autoware_agnocast_wrapper \
    /autoware_core/common/autoware_component_interface_specs \
    /autoware_core/common/autoware_geography_utils \
    /autoware_core/common/autoware_lanelet2_utils \
    /autoware_core/map/autoware_map_loader \
    /autoware_core/map/autoware_map_projection_loader \
  --packages-up-to autoware_map_loader autoware_map_projection_loader \
  --cmake-args -DBUILD_TESTING=OFF >/tmp/build.log 2>&1

cat >/tmp/map_projection_loader_params.yaml <<'PARAMS'
/**:
  ros__parameters:
    map_projector_info_path: "/maps/map_projector_info.yaml"
    lanelet2_map_path: ""
PARAMS

cat >/tmp/pointcloud_map_loader_params.yaml <<'PARAMS'
/**:
  ros__parameters:
    enable_whole_load: true
    enable_downsampled_whole_load: false
    enable_partial_load: true
    enable_selected_load: false
    leaf_size: 3.0
    pcd_paths_or_directory: ["/maps/pointcloud_map"]
    pcd_metadata_path: "/maps/pointcloud_map/pointcloud_map_metadata.yaml"
PARAMS

set +u
source /autoware_ws/install/setup.bash
set -u

ros2 run autoware_map_projection_loader autoware_map_projection_loader_node \
  --ros-args \
  --params-file /tmp/map_projection_loader_params.yaml >/tmp/map_projection_loader.log 2>&1 &
P1=$!

ros2 run autoware_map_loader autoware_pointcloud_map_loader \
  --ros-args \
  --params-file /tmp/pointcloud_map_loader_params.yaml \
  --remap output/pointcloud_map:=/map/pointcloud_map \
  --remap service/get_partial_pcd_map:=/map/get_partial_pointcloud_map \
  --remap service/get_selected_pcd_map:=/map/get_selected_pointcloud_map \
  --remap service/get_differential_pcd_map:=/map/get_differential_pointcloud_map \
  >/tmp/pointcloud_map_loader.log 2>&1 &
P2=$!

python3 - <<'PY'
import sys
import time

import rclpy
from autoware_map_msgs.msg import MapProjectorInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Waiter(Node):
    def __init__(self):
        super().__init__("autoware_map_smoke_test")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_msg = None
        self.proj_msg = None
        self.create_subscription(PointCloud2, "/map/pointcloud_map", self.on_map, qos)
        self.create_subscription(MapProjectorInfo, "/map/map_projector_info", self.on_proj, qos)

    def on_map(self, msg):
        self.map_msg = msg

    def on_proj(self, msg):
        self.proj_msg = msg


rclpy.init()
node = Waiter()
deadline = time.time() + 20.0
while time.time() < deadline and (node.map_msg is None or node.proj_msg is None):
    rclpy.spin_once(node, timeout_sec=0.5)

if node.proj_msg is None:
    print("ERROR: did not receive /map/map_projector_info", file=sys.stderr)
    sys.exit(2)

if node.map_msg is None:
    print("ERROR: did not receive /map/pointcloud_map", file=sys.stderr)
    sys.exit(3)

print(
    f"MAP_PROJECTOR_INFO projector_type={node.proj_msg.projector_type} "
    f"scale_factor={node.proj_msg.scale_factor}"
)
print(
    f"POINTCLOUD_MAP width={node.map_msg.width} height={node.map_msg.height} "
    f"frame_id={node.map_msg.header.frame_id} data_bytes={len(node.map_msg.data)}"
)

node.destroy_node()
rclpy.shutdown()
PY

echo "SERVICES"
ros2 service list | grep -E "/map/get_(partial|selected|differential)_pointcloud_map" || true

print_log_tail /tmp/map_projection_loader.log
print_log_tail /tmp/pointcloud_map_loader.log
EOF
