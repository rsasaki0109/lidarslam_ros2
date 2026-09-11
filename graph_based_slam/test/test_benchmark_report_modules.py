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

"""Focused tests for the benchmark report modules."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from lidarslam_benchmark_tools.lidarslam_tools.report_charts import line_chart_svg
from lidarslam_benchmark_tools.lidarslam_tools.report_diagnostics import collect_log_alerts
from lidarslam_benchmark_tools.lidarslam_tools.report_model import (
    as_bool,
    infer_reference_kind,
    run_quality,
    RunRecord,
)
from lidarslam_benchmark_tools.lidarslam_tools.trajectory_analysis import (
    associate_poses,
    Pose,
    unwrap_degrees,
)


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_legacy_entrypoint_exports_the_public_helpers():
    spec = importlib.util.spec_from_file_location(
        'generate_html_report', REPO_ROOT / 'scripts' / 'generate_html_report.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    expected = {
        'RunRecord', 'as_bool', 'load_record', 'run_quality',
        'build_aligned_series', 'line_chart_svg', 'build_page', 'main',
    }
    assert expected <= set(module.__all__)
    assert all(hasattr(module, name) for name in module.__all__)


def _record(**overrides) -> RunRecord:
    values = {
        'group': 'fixture', 'run': 'run_1', 'metrics_path': Path('metrics.json'),
        'bag': '', 'bag_name': '-', 'ape_rmse': 0.02, 'ape_median': 0.02,
        'ape_max': 0.04, 'lid_ok': True, 'glim_ok': True, 'lid_rtf': 1.0,
        'glim_rtf': 1.0, 'lid_wall': 1.0, 'glim_wall': 1.0,
        'reference_kind': 'ground_truth', 'reference_source': 'gt',
        'param_name': 'auto', 'lid_tum_path': None, 'glim_traj_path': None,
        'mtime': 0.0,
    }
    values.update(overrides)
    return RunRecord(**values)


def test_model_coercion_and_quality_rules():
    assert as_bool('yes') is True
    assert as_bool('off') is False
    assert as_bool('unknown') is None
    assert infer_reference_kind('GLIM fresh', '') == 'cross_validation'
    assert run_quality(_record()) == ('GOOD', 'good')
    assert run_quality(_record(lid_ok=False)) == ('BAD', 'bad')


def test_pose_association_and_angle_unwrap():
    def pose(stamp):
        return Pose(stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    ref, est = associate_poses([pose(1.0), pose(2.0)], [pose(1.01), pose(3.0)])
    assert [item.t for item in ref] == [1.0]
    assert [item.t for item in est] == [1.01]
    assert unwrap_degrees([179.0, -179.0, -178.0]) == [179.0, 181.0, 182.0]


def test_log_alerts_are_deduplicated_and_limited(tmp_path):
    (tmp_path / 'slam.log').write_text(
        'TF_OLD_DATA from the past for frame map\n'
        'TF_OLD_DATA from the past for frame map\n'
        'fatal: solver stopped\n',
        encoding='utf-8',
    )
    alerts = collect_log_alerts(tmp_path, limit=2)
    assert [alert['label'] for alert in alerts] == ['TF', 'FATAL']


def test_line_chart_escapes_labels():
    svg = line_chart_svg([0.0, 1.0], [0.0, 1.0], [0.1, 1.1], '<position>', 'm')
    assert svg.startswith('<svg')
    assert '&lt;position&gt;' in svg
    assert '<polyline' in svg
