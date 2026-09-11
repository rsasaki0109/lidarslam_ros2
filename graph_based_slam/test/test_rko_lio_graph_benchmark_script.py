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

"""CLI regression tests for the RKO-LIO graph benchmark wrapper."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_SCRIPT = REPO_ROOT / 'scripts' / 'run_rko_lio_graph_benchmark.sh'
COMPETITIVE_RKO_CONFIG = (
    REPO_ROOT / 'configs' / 'hilti2022' /
    'rko_lio_hilti2022_pandar_competitive_v2.yaml'
)
PIECEWISE_RKO_CONFIG = (
    REPO_ROOT / 'configs' / 'hilti2022' /
    'rko_lio_hilti2022_pandar_competitive_piecewise_v1.yaml'
)


def _run_benchmark(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(BENCHMARK_SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def _write_minimal_bag(tmp_path: Path) -> Path:
    bag_dir = tmp_path / 'demo_bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  starting_time:\n'
        '    nanoseconds_since_epoch: 0\n'
        '  duration:\n'
        '    nanoseconds: 1\n',
        encoding='utf-8',
    )
    return bag_dir


def test_rko_lio_benchmark_help_exits_successfully():
    result = _run_benchmark('--help')

    assert result.returncode == 0
    assert 'run_rko_lio_graph_benchmark.sh' in result.stderr
    assert '--skip-reference-gen' in result.stderr


def test_trajectory_only_mode_uses_full_dump_as_explicit_passthrough():
    script = BENCHMARK_SCRIPT.read_text(encoding='utf-8')
    assert 'find "$OUTPUT_DIR" -mindepth 2 -maxdepth 2' in script
    assert 'Authoritative full-rate trajectory:' in script
    assert 'benchmark expected exactly one authoritative full-rate TUM dump' in script
    assert 'Trajectory-only passthrough from full-rate dump' in script
    assert 'cp "${BACKEND_TUMS[0]}" "$RAW_TUM"' in script


def test_installed_launch_target_uses_ros2_package_and_file_arguments():
    script = BENCHMARK_SCRIPT.read_text(encoding='utf-8')

    assert 'RKO_LAUNCH_TARGET=(lidarslam rko_lio_slam.launch.py)' in script
    assert 'ros2 launch "${RKO_LAUNCH_TARGET[@]}"' in script
    assert 'RKO_LAUNCH_TARGET="lidarslam/rko_lio_slam.launch.py"' not in script


def test_ordinary_completion_does_not_treat_pending_as_v2_corruption():
    script = BENCHMARK_SCRIPT.read_text(encoding='utf-8')
    ordinary_branch = script.split(
        '    else\n      if [[ -s "$RKO_RESULT_TUM" ]]; then', 1)[1]
    ordinary_branch = ordinary_branch.split(
        '    fi\n\n    if [[ -n "$LAUNCH_PID" ]]', 1)[0]

    assert 'if offline_completion_recorded; then' in ordinary_branch
    assert 'v2 consumer evidence is invalid or malformed' not in ordinary_branch
    assert 'completion_state' not in ordinary_branch


def test_sparse_graph_path_is_densified_before_scoring():
    script = BENCHMARK_SCRIPT.read_text(encoding='utf-8')

    assert 'CORRECTED_SPARSE_TUM="${OUTPUT_DIR}/traj_corrected_sparse.tum"' in script
    assert 'densify_corrected_trajectory.py' in script
    assert '--corrected "$CORRECTED_SPARSE_TUM"' in script
    assert '--output "$CORRECTED_TUM"' in script
    assert 'dense corrected trajectory pose count differs' in script


def test_piecewise_candidate_changes_exactly_one_estimator_knob():
    baseline = yaml.safe_load(COMPETITIVE_RKO_CONFIG.read_text(encoding='utf-8'))
    candidate = yaml.safe_load(PIECEWISE_RKO_CONFIG.read_text(encoding='utf-8'))

    assert candidate['piecewise_gyro_deskew'] is True
    candidate_without_piecewise = dict(candidate)
    candidate_without_piecewise.pop('piecewise_gyro_deskew')
    assert candidate_without_piecewise == baseline


def test_quiet_completion_requires_substantial_bag_progress():
    """Require both near-end and minimum-progress quiet-completion guards."""
    script = BENCHMARK_SCRIPT.read_text(encoding='utf-8')

    assert 'COMPLETION_END_MARGIN_SECS=120' in script
    assert 'raw_tum_reached "$end_stamp" "$COMPLETION_END_MARGIN_SECS"' in script
    assert 'raw_tum_reached_fraction 0.8' in script
    assert '[[ -n "$end_stamp" ]]' in script


def test_rko_lio_benchmark_rejects_missing_option_value_before_realpath():
    result = _run_benchmark('--bag', '--skip-map-save')

    assert result.returncode == 2
    assert 'error: option requires a value: --bag' in result.stderr
    assert 'realpath' not in result.stderr


def test_rko_lio_benchmark_rejects_invalid_bool_without_usage_dump():
    result = _run_benchmark('--publish-static-tf', 'maybe')

    assert result.returncode == 2
    assert 'error: --publish-static-tf expects true or false.' in result.stderr
    assert 'Usage:' not in result.stderr


def test_rko_lio_benchmark_reports_missing_bag_without_realpath(tmp_path: Path):
    result = _run_benchmark(
        '--bag',
        str(tmp_path / 'missing_bag'),
        '--skip-reference-gen',
    )

    assert result.returncode == 2
    assert 'error: rosbag2 directory not found:' in result.stderr
    assert 'metadata.yaml' in result.stderr
    assert 'realpath' not in result.stderr


def test_rko_lio_benchmark_rejects_missing_metadata_before_ros2(tmp_path: Path):
    bag_dir = tmp_path / 'bag_without_metadata'
    bag_dir.mkdir()

    result = _run_benchmark('--bag', str(bag_dir), '--skip-reference-gen')

    assert result.returncode == 2
    assert 'error: metadata.yaml not found under' in result.stderr
    assert 'ros2 not found' not in result.stderr


def test_rko_lio_benchmark_rejects_param_file_before_ros2(tmp_path: Path):
    bag_dir = _write_minimal_bag(tmp_path)

    result = _run_benchmark(
        '--bag',
        str(bag_dir),
        '--lidarslam-param',
        str(tmp_path / 'missing_lidarslam.yaml'),
        '--skip-reference-gen',
    )

    assert result.returncode == 2
    assert 'error: lidarslam param file not found:' in result.stderr
    assert 'ros2 not found' not in result.stderr


def test_rko_lio_benchmark_rejects_output_file_before_ros2(tmp_path: Path):
    output_file = tmp_path / 'output_file'
    output_file.write_text('', encoding='utf-8')

    result = _run_benchmark(
        '--output-dir',
        str(output_file),
        '--skip-reference-gen',
    )

    assert result.returncode == 2
    assert 'error: output directory path is a file:' in result.stderr
    assert 'ros2 not found' not in result.stderr
