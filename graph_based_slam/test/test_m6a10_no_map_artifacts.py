# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Fail-closed source and shell fixtures for M6a10 no-map replay mode."""

from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / 'scripts' / 'run_rko_lio_graph_benchmark.sh'
WRAPPER = ROOT / 'scripts' / 'ours_container_gt_blind_run.sh'
LAUNCH = ROOT / 'lidarslam' / 'launch' / 'rko_lio_slam.launch.py'
GRAPH_COMPONENT = ROOT / 'graph_based_slam' / 'src' / 'graph_based_slam_component.cpp'


def _map_assertion_function() -> str:
    source = RUNNER.read_text(encoding='utf-8')
    start = source.index('assert_no_map_artifacts() {')
    end = source.index('\n}\n', start) + 3
    return source[start:end]


def _run_map_assertion(output: Path) -> subprocess.CompletedProcess[str]:
    script = f"""\
set -euo pipefail
OUTPUT_DIR={str(output)!r}
{_map_assertion_function()}
assert_no_map_artifacts
"""
    return subprocess.run(
        ['bash', '-c', script],
        check=False,
        capture_output=True,
        text=True,
    )


def test_no_map_assertion_accepts_trajectory_and_phase_outputs(tmp_path: Path):
    output = tmp_path / 'out'
    (output / 'm6a_0').mkdir(parents=True)
    (output / 'm6a_0' / 'm6a_tum_0.txt').write_text('trajectory\n')
    (output / 'phase_evidence.json').write_text('{}\n')

    result = _run_map_assertion(output)

    assert result.returncode == 0


def test_no_map_assertion_rejects_map_bundle_and_pointcloud_tree(tmp_path: Path):
    output = tmp_path / 'out'
    output.mkdir()
    (output / 'map.pcd').write_bytes(b'forbidden')
    (output / 'pointcloud_map').mkdir()

    result = _run_map_assertion(output)

    assert result.returncode == 125
    assert 'forbidden map artifact' in result.stderr


def test_skip_map_is_an_explicit_benchmark_only_launch_override():
    launch = LAUNCH.read_text(encoding='utf-8')
    wrapper = WRAPPER.read_text(encoding='utf-8')

    assert "M6A10_BENCHMARK_NO_MAP_ARTIFACTS'" in launch
    assert "overrides['use_save_map_in_loop'] = False" in launch
    assert "overrides['save_pose_graph_path'] = ''" in launch
    assert 'export M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1' in wrapper
    assert 'unset M6A10_BENCHMARK_NO_MAP_ARTIFACTS' in wrapper


def test_v1_failure_origin_is_loop_pose_adjustment_map_save_branch():
    component = GRAPH_COMPONENT.read_text(encoding='utf-8')

    assert 'doPoseAdjustment(map_array_msg, loop_edges, use_save_map_in_loop_)' in component
    assert 'if (do_save_map) {' in component
    assert 'saveGridDividedMap(map_to_save);' in component
    assert 'writeMapBundleArtifacts(map_array_msg, loop_edges, opt_result.poses);' in component
    assert 'writeDegeneracyReport();' in component


def test_skip_map_runner_verifies_after_launch_termination():
    source = RUNNER.read_text(encoding='utf-8')
    termination = source.index('LAUNCH_PID=""\nLAUNCH_PGID=""')
    assertion = source.index('\n  assert_no_map_artifacts\n', termination)

    assert assertion > termination
    assert source.index('assert_no_map_artifacts() {') < assertion
    assert "-name 'pointcloud_map'" in source
    assert "-name 'map_save.log'" in source
    assert "-name 'pose_graph.g2o'" in source
