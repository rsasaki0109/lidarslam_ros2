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
Static/dry checks for the additive v11 bounded-end-gap delta.

This test never builds Docker images, opens a bag, starts ROS, or executes a
formal replay.  It checks that the delta is applicable after v10 and that the
host-only synthetic executable covers the v10 regressions plus v11 bounds.
"""

import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch'
SELFTEST = ROOT / 'tools/m6a10_terminal_bounded_end_gap_selftest.cpp'
V10_PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch'


def _write_post_v10_fixture(repo: Path) -> None:
    """
    Materialize only the post-v10 contexts needed by the delta.

    The production source is supplied by the Docker build, not this repo.
    Staging this exact post-v10-shaped fixture lets ``git apply --check``
    validate the real patch format and hunk contexts without opening Docker,
    ROS, a bag, or any runtime input.
    """
    include = repo / 'include'
    include.mkdir(parents=True)

    terminal = ['// post-v10 fixture'] * 36
    terminal.extend(
        [
            '  static const char *contract_version()',
            '  {',
            '    return "m6a10-online-compute-v3-terminal-support-context";',
            '  }',
            '  enum class Topic { Lidar, Imu, Image };',
            '',
        ]
    )
    terminal.extend(['// post-v10 fixture'] * (594 - len(terminal)))
    terminal.extend(
        [
            '        !boundary_.quiescence_observed || boundary_.in_flight ||',
            '        estimator_in_flight_ || unit_open_ ||',
            '        !std::isfinite(boundary_.timestamp_seconds) ||',
            '        boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||',
            '        boundary_.sequence < 0 || boundary_.source.empty())',
            '      return false;',
            '    if (boundary_.completed_counts.lidar + '
            'buffers_.find("lidar")->second.count != received_.lidar ||',
        ]
    )
    (include / 'm6a10_terminal_support_context.h').write_text(
        '\n'.join(terminal) + '\n', encoding='utf-8'
    )

    consumer = ['// post-v10 fixture'] * 145
    consumer.extend(
        [
            '    enabled_ = contract_value == "m6a10-online-compute-v2" ||',
            '      contract_value == "m6a10-online-compute-v3-terminal-support-context";',
            '    if (!enabled_) return;',
        ]
    )
    (include / 'm6a10_consumer_evidence.h').write_text(
        '\n'.join(consumer) + '\n', encoding='utf-8'
    )

    subprocess.run(['git', 'init', '--quiet', str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), 'add', 'include'], check=True)


def test_v11_delta_is_new_and_targets_only_required_files():
    assert PATCH.is_file()
    text = PATCH.read_text(encoding='utf-8')
    syntax = subprocess.run(
        ['git', 'apply', '--stat', str(PATCH)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    assert syntax.stdout.count('include/m6a10_terminal_support_context.h') == 1
    assert syntax.stdout.count('include/m6a10_consumer_evidence.h') == 1
    assert 'src/LIVMapper.cpp' not in syntax.stdout

    diff_paths = [line.split()[2] for line in text.splitlines() if line.startswith('diff --git ')]
    assert diff_paths == [
        'a/include/m6a10_terminal_support_context.h',
        'a/include/m6a10_consumer_evidence.h',
    ]
    assert text.count('diff --git ') == 2
    assert 'v11_enabled' not in text
    assert 'src/LIVMapper.cpp' not in text

    # Exactly three semantic edits: one contract identifier, one contradictory
    # boundary predicate, and one consumer activation identifier.
    assert text.count('-    return "m6a10-online-compute-v3-terminal-support-context";') == 1
    assert text.count('+    return "m6a10-online-compute-v4-terminal-bounded-end-gap";') == 1
    assert (
        text.count('-        boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||')
        == 1
    )
    assert '+        boundary_.timestamp_seconds < required_end_timestamp_seconds_ ||' not in text
    assert (
        text.count('+      contract_value == "m6a10-online-compute-v4-terminal-bounded-end-gap";')
        == 1
    )

    with tempfile.TemporaryDirectory(prefix='m6a10-v11-post-v10-') as tmp:
        repo = Path(tmp) / 'post-v10-source'
        _write_post_v10_fixture(repo)
        checked = subprocess.run(
            ['git', '-C', str(repo), 'apply', '--check', '--recount', str(PATCH)],
            text=True,
            capture_output=True,
        )
        assert checked.returncode == 0, checked.stderr

    # No base/v10 file is rewritten by this task.
    assert V10_PATCH.is_file()
    assert hashlib.sha256(V10_PATCH.read_bytes()).hexdigest() == (
        '155bae4e37eac7d6220baedd861b9110b6139ee2b384d5b90db78ac24a6733c6'
    )


def test_selftest_has_all_cases_and_compiles():
    text = SELFTEST.read_text(encoding='utf-8')
    names = [
        'zero_backlog',
        'post_boundary_imu_image_tail',
        'residual_lidar',
        'pre_boundary_tail',
        'discard_clear',
        'active_inflight',
        'same_rpc_stability',
        'missing_eof',
        'bounded_end_gap',
        'excessive_end_gap',
        'future_boundary',
    ]
    for name in names:
        assert f'"{name}"' in text
    assert 'm6a10-online-compute-v4-terminal-bounded-end-gap' in text
    assert 'o.trajectory_end_gap < 0.0' in text
    assert 'o.trajectory_end_gap > kMaximumGap' in text
    assert 'kRequiredEnd - o.trajectory_last_timestamp' in text
    compiler = shutil.which('g++')
    if compiler is None:
        return
    with tempfile.TemporaryDirectory(prefix='m6a10-v11-selftest-') as tmp:
        binary = Path(tmp) / 'selftest'
        subprocess.run(
            [
                compiler,
                '-std=c++17',
                '-Wall',
                '-Wextra',
                '-Werror',
                '-O2',
                str(SELFTEST),
                '-o',
                str(binary),
            ],
            cwd=ROOT,
            check=True,
        )
        result = subprocess.run(
            [str(binary)], cwd=ROOT, text=True, capture_output=True, check=True
        )
        assert 'M6A10_SYNTHETIC_ALL PASS' in result.stdout
        for name in names:
            assert re.search(rf'M6A10_SYNTHETIC_CASE {name} .* gate=PASS', result.stdout)
