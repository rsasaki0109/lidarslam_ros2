# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the Applanix + Velodyne open-data GNSS smoke flow."""

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SMOKE_SCRIPT = (
    REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh'
)
OVERLAY_SCRIPT = (
    REPO_ROOT / 'scripts' / 'prepare_velodyne_pointcloud_overlay.sh'
)


def test_applanix_velodyne_smoke_script_uses_packet_conversion_and_gnss_sidecar():
    """The wrapper should convert packets and generate GNSS/IMU sidecars."""
    script = SMOKE_SCRIPT.read_text(encoding='utf-8')

    assert 'convert_applanix_gsof_to_navsatfix_bag.py' in script
    assert 'convert_applanix_gsof_to_imu_bag.py' in script
    assert 'velodyne_transform_node' in script
    assert 'select_rosbag_topic.py' in script
    assert '"/front/")"' in script
    assert '--gsof49-topic "${GSOF49_TOPIC}"' in script
    assert '--gsof50-topic "${GSOF50_TOPIC}"' in script
    assert '--output-topic "${GNSS_TOPIC}"' in script
    assert '--output-topic "${IMU_TOPIC}"' in script
    assert '--frame-id "${ROBOT_FRAME_ID}"' in script
    assert '--qos-profile-overrides-path "${QOS_FILE}"' in script
    assert '"gnss_topic:=${GNSS_TOPIC}" \\' in script
    assert '"imu_topic:=${IMU_TOPIC}" \\' in script
    assert '"input_cloud:=${POINTS_TOPIC}" \\' in script
    assert '"publish_static_tf:=false" \\' in script
    assert 'POINTS_TOPIC="/open_data/velodyne_points"' in script
    assert '--use-imu BOOL' in script
    assert '--imu-pose-prediction BOOL' in script
    assert 'IMU_TRANSLATION_DESKEW="false"' in script
    assert 'IMU_POSE_PREDICTION="false"' in script
    assert 'if [[ "${USE_IMU,,}" == "true" ]]; then' in script
    assert 'prepare_rosbag2_playback.py' in script
    assert 'stage_playback_bag "${BAG_PATH}" "${SAVE_DIR}" MAIN_PLAY_BAG' in script
    assert 'ros2 bag play "${MAIN_PLAY_BAG}"' in script
    assert 'cleanup_playback_staging' in script


def test_applanix_velodyne_smoke_script_supports_overlay_bootstrap():
    """The wrapper should be able to bootstrap velodyne_pointcloud."""
    script = SMOKE_SCRIPT.read_text(encoding='utf-8')

    assert 'prepare_velodyne_pointcloud_overlay.sh' in script
    assert '--skip-prepare-overlay' in script
    assert 'ensure_velodyne_overlay' in script
    assert '--check >/dev/null 2>&1' in script
    assert 'resolve_velodyne_msg_dir' in script
    assert 'velodyne_msgs definitions not found' in script
    assert 'velodyne_msgs/msg/VelodyneScan' in script
    assert '${overlay_dir}/src/velodyne/velodyne_msgs/msg' in script
    assert 'default_calibration_for_model' in script
    assert 'VLP16)' in script
    assert '32C|VLP32C)' in script
    assert 'VLS128)' in script


def test_applanix_profile_reports_rosbags_dependency_before_launch():
    script = SMOKE_SCRIPT.read_text(encoding='utf-8')

    assert 'require_rosbags' in script
    assert "requires the Python package 'rosbags'" in script
    assert 'docs/distribution.md#profile-specific-extras' in script


def test_overlay_preparation_script_stays_minimal_and_public():
    """The overlay helper should pull only the minimum public repos/packages."""
    script = OVERLAY_SCRIPT.read_text(encoding='utf-8')

    assert 'https://github.com/ros-drivers/velodyne.git' in script
    assert 'https://github.com/ros/diagnostics.git' in script
    assert 'https://github.com/ros/angles.git' in script
    assert '--check' in script
    assert 'overlay_is_ready' in script
    assert 'velodyne_transform_node" ]]' in script
    assert 'params/VLP16db.yaml" ]]' in script
    assert 'colcon --log-base "${OVERLAY_DIR}/log" build \\\n' in script
    assert (
        '--packages-select angles diagnostic_updater velodyne_msgs '
        'velodyne_pointcloud'
    ) in script
    assert (
        'set +u\n'
        '# shellcheck source=/dev/null\n'
        'source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"\n'
        'set -u\n'
    ) in script


def test_overlay_check_rejects_partial_install(tmp_path):
    """A partial colcon install must not be mistaken for a ready overlay."""
    setup = tmp_path / 'install' / 'setup.bash'
    setup.parent.mkdir(parents=True)
    setup.touch()

    result = subprocess.run(
        [
            'bash', str(OVERLAY_SCRIPT),
            '--overlay-dir', str(tmp_path),
            '--check',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert 'velodyne overlay is incomplete' in result.stderr


def test_overlay_check_accepts_complete_install(tmp_path):
    """Readiness should cover messages, runtime, and calibration."""
    required_files = [
        tmp_path / 'install' / 'setup.bash',
        tmp_path / 'install' / 'velodyne_msgs' / 'share' /
        'velodyne_msgs' / 'msg' / 'VelodyneScan.msg',
        tmp_path / 'install' / 'velodyne_pointcloud' / 'share' /
        'velodyne_pointcloud' / 'params' / 'VLP16db.yaml',
    ]
    executable = (
        tmp_path / 'install' / 'velodyne_pointcloud' / 'lib' /
        'velodyne_pointcloud' / 'velodyne_transform_node'
    )
    for path in [*required_files, executable]:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    executable.chmod(0o755)

    result = subprocess.run(
        [
            'bash', str(OVERLAY_SCRIPT),
            '--overlay-dir', str(tmp_path),
            '--check',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert f'overlay_ready: {tmp_path}' in result.stdout
