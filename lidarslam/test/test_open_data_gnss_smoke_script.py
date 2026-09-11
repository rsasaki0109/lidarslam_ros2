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

"""Regression tests for the open-data GNSS smoke workflow script."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_open_data_gnss_smoke.sh'


def test_smoke_script_launches_with_gnss_topic_override():
    """The smoke workflow should forward gnss_topic into the launch call."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert '"gnss_topic:=${GNSS_TOPIC}" \\' in script


def test_smoke_script_autodetects_navsatfix_topic():
    """The smoke workflow should auto-detect NavSatFix when not specified."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'GNSS_TOPIC_SOURCE="${BAG_PATH}"' in script
    assert 'GNSS_TOPIC_SOURCE="${GNSS_BAG}"' in script
    assert (
        'GNSS_TOPIC="$(detect_topic_by_type "${GNSS_TOPIC_SOURCE}" '
        '"sensor_msgs/msg/NavSatFix")"' in script
    )
    assert '[[ -n "${GNSS_TOPIC}" ]] || die "failed to detect NavSatFix topic"' in script


def test_smoke_script_reports_profile_specific_rosbags_dependency():
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'require_rosbags' in script
    assert "requires the Python package 'rosbags'" in script
    assert 'docs/distribution.md#profile-specific-extras' in script
