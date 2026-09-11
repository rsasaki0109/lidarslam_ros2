"""Shared immutable dependency-install command identity for release legs."""

from __future__ import print_function

import hashlib


DEPENDENCY_INSTALL_COMMAND = (
    "set -e; apt-get update; "
    "DEBIAN_FRONTEND=noninteractive apt-get install -y "
    "git build-essential cmake pkg-config python3-rosdep "
    "python3-colcon-common-extensions; "
    "if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi; "
    "rosdep update; "
    "rosdep install -r -y --from-paths "
    "/workspace/src/lidarslam_plugin_interfaces "
    "/workspace/src/lidarslam_default_plugins "
    "/workspace/src/lidarslam_fake_registration_plugins "
    "/workspace/src/lidarslam_registration_loader "
    "/workspace/src/lidarslam_msgs "
    "/workspace/src/scanmatcher "
    "/workspace/src/graph_based_slam --ignore-src --rosdistro " + "$" + "{ROS_DISTRO}"
)


def dependency_install_command_sha256():
    """Return the hash of the exact command sent to the container shell."""
    return hashlib.sha256(DEPENDENCY_INSTALL_COMMAND.encode("utf-8")).hexdigest()


if __name__ == "__main__":
    raise SystemExit("registration_plugin_dependency_contract is a library module; import it instead of running it directly.")
