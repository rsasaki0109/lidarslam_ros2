#!/usr/bin/env bash
# Source ROS + the installed lidarslam product, then run the given command.
# ROS-generated setup files read optional variables before assigning them, so
# enable nounset only after both environments have been loaded.
set -eo pipefail

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f /lidarslam_ws/install/setup.bash ]]; then
  echo "error: installed lidarslam runtime is missing" >&2
  exit 127
fi
# shellcheck source=/dev/null
source /lidarslam_ws/install/setup.bash

set -u
exec "$@"
