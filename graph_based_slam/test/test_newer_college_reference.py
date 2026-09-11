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

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_newer_college_reference.py'


def _run(tmp_path: Path, csv_text: str) -> tuple[subprocess.CompletedProcess, Path, Path]:
    source = tmp_path / 'maths_hard_gt.csv'
    source.write_text(csv_text, encoding='utf-8')
    calibration = tmp_path / 'collection3_calibration.yaml'
    calibration.write_text('base_to_lidar: [1, 0, 0, 0, 0.1, 0, 0]\n', encoding='utf-8')
    output = tmp_path / 'maths_hard_gt.tum'
    metadata = tmp_path / 'maths_hard_reference.json'
    result = subprocess.run(
        [
            'python3', str(SCRIPT),
            '--csv', str(source),
            '--output', str(output),
            '--metadata', str(metadata),
            '--calibration', str(calibration),
            '--body-to-reference-x', '-0.1',
            '--body-to-reference-y', '0',
            '--body-to-reference-z', '0',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )
    return result, output, metadata


def test_official_positional_csv_converts_to_tum_and_metadata(tmp_path: Path):
    """The official seconds/nanoseconds + pose layout is frozen faithfully."""
    result, output, metadata = _run(
        tmp_path,
        '\n'.join([
            'seconds,nanoseconds,x,y,z,qx,qy,qz,qw',
            '1700000000,100,1,2,3,0,0,0,1',
            '1700000000,200,2,3,4,0,0,0,1',
            '1700000000,300,3,4,5,0,0,0,1',
            '',
        ]),
    )
    assert result.returncode == 0, result.stderr
    lines = output.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 3
    assert lines[0].endswith('1 2 3 0 0 0 1')
    document = json.loads(metadata.read_text(encoding='utf-8'))
    assert document['source'] == 'official_newer_college_math_hard_icp_map_gt_csv'
    assert document['pose_count'] == 3
    assert document['reference_point_frame'] == 'Base'
    assert document['body_to_reference_translation_m'] == {
        'x': -0.1, 'y': 0.0, 'z': 0.0,
    }
    assert len(document['source_csv_sha256']) == 64
    assert len(document['reference_translation_source_sha256']) == 64
    assert len(document['reference_tum_sha256']) == 64


def test_non_monotonic_reference_fails_closed(tmp_path: Path):
    """Timestamp reversal must not create release-eligible evidence."""
    result, output, metadata = _run(
        tmp_path,
        '\n'.join([
            '1700000000,100,1,2,3,0,0,0,1',
            '1700000000,90,2,3,4,0,0,0,1',
            '1700000000,300,3,4,5,0,0,0,1',
            '',
        ]),
    )
    assert result.returncode == 2
    assert 'timestamps must be strictly increasing' in result.stderr
    assert not output.exists()
    assert not metadata.exists()


def test_existing_reference_is_never_overwritten(tmp_path: Path):
    """Reference generation is immutable by default."""
    first, output, metadata = _run(
        tmp_path,
        '\n'.join([
            '1700000000,100,1,2,3,0,0,0,1',
            '1700000000,200,2,3,4,0,0,0,1',
            '1700000000,300,3,4,5,0,0,0,1',
            '',
        ]),
    )
    assert first.returncode == 0, first.stderr
    before = (output.read_bytes(), metadata.read_bytes())
    second, _, _ = _run(
        tmp_path,
        '\n'.join([
            '1700000001,100,9,9,9,0,0,0,1',
            '1700000001,200,9,9,9,0,0,0,1',
            '1700000001,300,9,9,9,0,0,0,1',
            '',
        ]),
    )
    assert second.returncode == 2
    assert 'refusing to overwrite reference artifacts' in second.stderr
    assert before == (output.read_bytes(), metadata.read_bytes())
