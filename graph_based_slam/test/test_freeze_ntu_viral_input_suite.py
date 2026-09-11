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

import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'freeze_suite', ROOT / 'scripts/freeze_ntu_viral_input_suite.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _fixture(tmp_path):
    prereg = tmp_path / 'prereg.json'
    prereg.write_text('{}')
    slots = {}
    manifests = []
    common = {key: 'a' * 64 for key in MODULE.CONSISTENT_HASHES}
    for index in range(1, 4):
        slot = f'holdout_{index}'
        sequence = f'seq_{index}'
        slots[slot] = {
            'sequence': sequence, 'archive_expected_bytes': 100 + index,
            'archive_expected_md5': f'{index}' * 32,
            'ground_truth_sha256': f'{index}' * 64}
        path = tmp_path / f'{sequence}.json'
        path.write_text(json.dumps({
            'status': 'frozen', 'slot': slot, 'sequence': sequence,
            'hashes': {
                **common, 'official_archive_bytes': 100 + index,
                'official_archive_md5': f'{index}' * 32,
                'reference_sha256': f'{index}' * 64,
                'raw_rosbag1_sha256': f'{index + 3}' * 64,
                'canonical_rosbag2_tree_sha256': f'{index + 6}' * 64,
                'semantic_report_sha256': f'{index + 2}' * 64}}))
        manifests.append(path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({'competitive_slam_profile': {
        'holdout_selection_preregistration_sha256': MODULE.sha256(prereg),
        'datasets': {'holdout_slots': slots}}}))
    return profile, prereg, manifests


def test_complete_consistent_suite_is_frozen(tmp_path):
    profile, prereg, manifests = _fixture(tmp_path)
    result = MODULE.freeze(profile, prereg, manifests)
    assert result['status'] == 'frozen'
    assert set(result['holdouts']) == {
        'holdout_1', 'holdout_2', 'holdout_3'}


def test_inconsistent_track_config_is_rejected(tmp_path):
    profile, prereg, manifests = _fixture(tmp_path)
    document = json.loads(manifests[1].read_text())
    document['hashes']['rko_lio_param_sha256'] = 'b' * 64
    manifests[1].write_text(json.dumps(document))
    with pytest.raises(ValueError, match='inconsistent shared hash'):
        MODULE.freeze(profile, prereg, manifests)
