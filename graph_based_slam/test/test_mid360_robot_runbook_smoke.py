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

"""Smoke tests for the MID-360 robot runbook script."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'smoke_mid360_robot_runbook.sh'


def test_mid360_robot_runbook_smoke_script(tmp_path: Path):
    work_dir = tmp_path / 'work'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash',
            str(SCRIPT_PATH),
            '--work-dir',
            str(work_dir),
            '--output-dir',
            str(output_dir),
            '--keep-work-dir',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert 'MID-360 robot runbook smoke: PASS' in result.stdout
    assert readiness['status'] == 'PASS'
    assert math.isclose(
        readiness['bag_diagnostics']['topics']['pointcloud']['metadata_rate_hz'],
        10.0,
        rel_tol=0.02,
    )
    assert math.isclose(
        readiness['bag_diagnostics']['topics']['imu']['metadata_rate_hz'],
        100.0,
        rel_tol=0.02,
    )
    assert (output_dir / 'mid360_robot_session_dashboard.html').is_file()
    assert (output_dir / 'mid360_robot_field_session.json').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()
    assert (output_dir / 'mid360_robot_recording_check.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_field_record_plan.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_record_record_plan.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_record_profile.yaml').is_file()
    assert (work_dir / 'recordings' / 'smoke_record' / 'metadata.yaml').is_file()
