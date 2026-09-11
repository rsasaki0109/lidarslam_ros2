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


"""
Static regression gates for the v12 no-input startup payload.

The payload is never sourced or executed here. These tests only run Bash
syntax validation and inspect the process-lifecycle contract.
"""

from __future__ import annotations

from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PAYLOAD = ROOT / 'scripts/fast_livo2_m6a10_v12_no_input_container_payload.sh'


def _text() -> str:
    assert PAYLOAD.is_file()
    return PAYLOAD.read_text(encoding='utf-8')


def test_payload_is_executable_and_bash_valid():
    assert PAYLOAD.stat().st_mode & 0o111
    checked = subprocess.run(
        ['bash', '-n', str(PAYLOAD)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert checked.returncode == 0, checked.stderr


def test_pid_capture_is_immediate_and_wrong_assignment_is_forbidden():
    text = _text()
    for command, variable in (
        ('roscore >"${OUT_DIR}/roscore.log" 2>&1 &', 'ROSCORE_PID'),
        (
            'roslaunch fast_livo mapping_ouster_ntu.launch '
            'rviz:=false >"${OUT_DIR}/mapper.log" 2>&1 &',
            'MAPPER_PID',
        ),
    ):
        pattern = re.escape(command) + r'\n' + re.escape(variable) + r'=\$!'
        assert re.search(pattern, text), f'missing immediate PID capture for {variable}'
    assert '& MAPPER_PID' not in text
    assert '& ROSCORE_PID' not in text
    assert 'MAPPER_PID=$!' in text
    assert 'ROSCORE_PID=$!' in text


def test_payload_has_ready_wait_and_safe_cleanup_for_both_processes():
    text = _text()
    assert 'rosparam list' in text
    assert 'rosparam set use_sim_time true' in text
    assert 'rosservice list' in text
    assert 'trap cleanup EXIT' in text
    assert "trap 'exit 130' INT" in text
    assert "trap 'exit 143' TERM" in text
    assert 'for pid in "${MAPPER_PID}" "${ROSCORE_PID}"' in text
    assert 'kill -0 "${pid}"' in text
    assert 'kill -TERM "${pid}"' in text
    assert 'wait "${pid}"' in text
    for service in (
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/consumer_eof',
        '/m6a10/consumer_finalize',
        '/m6a10/terminal_status',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ):
        assert service in text


def test_payload_has_no_input_or_evaluation_surface_reference():
    text = _text().lower()
    for forbidden in ('rosbag', 'ground_truth', 'scorer', 'map_save', '/input/'):
        assert forbidden not in text
