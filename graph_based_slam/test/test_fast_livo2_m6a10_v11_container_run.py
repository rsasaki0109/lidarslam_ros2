#!/usr/bin/env python3

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
Static contract tests for the v11 container wrapper candidate.

The wrapper is never invoked here: only its normalized source, executable bit,
and Bash syntax are checked.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
V10 = ROOT / 'scripts/fast_livo2_m6a10_v10_container_run.sh'
V11 = ROOT / 'scripts/fast_livo2_m6a10_v11_container_run.sh'
PROFILE_SHA256 = 'e8980198e52604bb7dca0d1c1ecb91b6ce4f8b95b1dea3bb80ad4db8cd382296'


def _v10_text() -> str:
    return V10.read_text(encoding='utf-8')


def _v11_text() -> str:
    return V11.read_text(encoding='utf-8')


def _normalized_v10_copy() -> str:
    value = _v10_text()
    replacements = (
        (
            '# Inside-container FAST-LIVO2 M6a10 v10 runtime entrypoint.',
            '# Inside-container FAST-LIVO2 M6a10 v11 runtime entrypoint.',
        ),
        (
            "EXPECTED_CONTRACT='m6a10-online-compute-v3-terminal-support-context'",
            "EXPECTED_CONTRACT='m6a10-online-compute-v4-terminal-bounded-end-gap'",
        ),
        (
            "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/"
            "fast_livo2_m6a10_v10_formal.yaml'",
            "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/"
            "fast_livo2_m6a10_v11_formal.yaml'",
        ),
        (
            "EXPECTED_PROFILE_SHA256='f706f41b0a985347ff98e4953532590c7a9c8c2d75db"
            "355e53cfb558f95bc459'",
            f"EXPECTED_PROFILE_SHA256='{PROFILE_SHA256}'",
        ),
    )
    for old, new in replacements:
        assert value.count(old) == 1, old
        value = value.replace(old, new, 1)
    return value


def test_v10_wrapper_is_unchanged_and_v11_diff_is_limited_to_identity():
    assert hashlib.sha256(V10.read_bytes()).hexdigest() == (
        '678b46402e441fcdc306140f104a878c564ec3e0e644957501bae431e041535b'
    )
    assert _v11_text() == _normalized_v10_copy()
    assert "EXPECTED_CONTRACT='m6a10-online-compute-v4-terminal-bounded-end-gap'" in _v11_text()
    assert (
        "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'"
        in _v11_text()
    )
    assert f"EXPECTED_PROFILE_SHA256='{PROFILE_SHA256}'" in _v11_text()


def test_v11_wrapper_is_executable_and_bash_syntax_valid():
    assert V11.stat().st_mode & 0o111
    checked = subprocess.run(['bash', '-n', str(V11)], check=False, capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr


def test_v11_keeps_exact_input_timing_feeder_and_evidence_contracts():
    text = _v11_text()
    required = (
        "EXPECTED_PHASE_MODE='unpaced_ack'",
        "EXPECTED_BAG_PATH='/input/ntu_viral.bag'",
        "EXPECTED_BAG_BYTES='11290464091'",
        "EXPECTED_BAG_SHA256='5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310'",
        "EXPECTED_MESSAGES='236687'",
        "EXPECTED_LIDAR_MESSAGES='5793'",
        "EXPECTED_IMU_MESSAGES='225102'",
        "EXPECTED_IMAGE_MESSAGES='5792'",
        "EXPECTED_END_TIMESTAMP='1623491515.148352'",
        "EXPECTED_END_GAP='0.25'",
        "EXPECTED_MIN_POLL='0.05'",
        "EXPECTED_CALLBACK_LATENCY='0.25'",
        "EXPECTED_BACKLOG='1'",
        "EXPECTED_SENSOR_DURATION='579.278127298'",
        "EXPECTED_TIMING_CONTRACT='m6a10-online-compute-v3-timing-v1'",
        "EXPECTED_FEEDER_SHA256='1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562"
        "f46d00af9691831'",
        'consumer_evidence.json',
        'callback_consumer_evidence.json',
        'online_compute_timing.json',
        'consumer_evidence.sha256',
        'M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE',
        'M6A10_CONSUMER_EVIDENCE',
        'M6A10_ONLINE_TIMING_EVIDENCE',
    )
    for marker in required:
        assert marker in text, marker
