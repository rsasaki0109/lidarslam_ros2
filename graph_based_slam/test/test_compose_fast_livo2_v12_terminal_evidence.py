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
Host-only v12 compositor tests.

The compositor receives a binder-produced document.  The synthetic source
files are created by the companion binder fixture, but the compositor is
never given their paths as inputs and never parses them directly.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml'
PROFILE_SHA = '675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d'
COMPOSITOR_SCRIPT = ROOT / 'scripts/compose_fast_livo2_v12_terminal_evidence.py'
BINDER_SCRIPT = ROOT / 'scripts/bind_fast_livo2_v12_consumer_evidence.py'


def _load(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


BINDER = _load('m6a10_v12_binder_for_compositor', BINDER_SCRIPT)
COMPOSITOR = _load('m6a10_v12_compositor', COMPOSITOR_SCRIPT)
BINDER_FIXTURES = _load(
    'm6a10_v12_binder_fixtures',
    ROOT / 'graph_based_slam/test/test_bind_fast_livo2_v12_consumer_evidence.py',
)


def _bound_fixture(tmp_path: Path) -> tuple[Path, dict[str, Path]]:
    source_dir = tmp_path / 'sources'
    paths, _ = BINDER_FIXTURES._fixture(source_dir)
    bound = tmp_path / 'host_bound.json'
    BINDER.bind_consumer_evidence(
        feeder_path=paths['feeder'],
        callback_path=paths['callback'],
        terminal_path=paths['terminal'],
        timing_path=paths['timing'],
        output_path=bound,
        profile_path=PROFILE,
        expected_profile_sha256=PROFILE_SHA,
    )
    return bound, paths


def _load_bound(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(value, dict)
    path.chmod(0o644)
    return value


def _rewrite_bound(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def test_positive_v12_composition_separates_transport_terminal_and_timing(tmp_path):
    bound, _ = _bound_fixture(tmp_path)
    output = tmp_path / 'composed.json'
    result = COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound, output_path=output)
    assert result['status'] == 'pass'
    assert result['contract_id'] == COMPOSITOR.COMPOSED_CONTRACT
    assert result['transport']['transport_contract_version'] == COMPOSITOR.TRANSPORT_CONTRACT
    assert result['transport']['transport_outstanding_at_drain'] == 0
    assert result['terminal_support_context']['contract_version'] == COMPOSITOR.CONSUMER_CONTRACT
    assert result['terminal_support_context']['count_conservation_passed'] is True
    assert result['terminal_support_context']['residual_lidar_forbidden'] is True
    assert result['timing']['after_transport_and_terminal_validation'] is True
    assert result['timing']['online_compute_rtf'] >= 0.0
    assert result['safety'] == {
        'bag_opened': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'map_saved': False,
        'formal_replay_started': False,
    }
    written = json.loads(output.read_text(encoding='utf-8'))
    assert written['status'] == 'pass'
    assert output.stat().st_mode & 0o222 == 0


@pytest.mark.parametrize(
    'mutation',
    [
        'callback_authority',
        'callback_counts',
        'callback_transport',
        'terminal_equal_lidar',
        'terminal_preboundary',
        'terminal_gap',
        'terminal_residual_lidar',
        'timing_rtf',
        'safety',
    ],
)
def test_bound_observation_mutations_reject_and_seal_invalid_receipt(tmp_path, mutation):
    bound, _ = _bound_fixture(tmp_path)
    value = _load_bound(bound)
    if mutation == 'callback_authority':
        value['transport']['raw']['input'] = {'bag_path': 'wrong'}
    elif mutation == 'callback_counts':
        value['transport']['raw']['consumer']['received_messages'] = 1
    elif mutation == 'callback_transport':
        value['transport']['raw']['transport_contract_version'] = 'wrong'
    elif mutation == 'terminal_equal_lidar':
        value['terminal_support_context']['raw']['terminal_support_context'][
            'classification'
        ] = 'wrong'
    elif mutation == 'terminal_preboundary':
        record = value['terminal_support_context']['raw']['terminal_support_context']['records'][0]
        record.update(
            {
                'timestamp_seconds': BINDER_FIXTURES.BOUNDARY - 0.01,
                'post_boundary': False,
                'equal_to_completed_boundary': False,
                'reason_code': 'strictly_after_completed_boundary',
            }
        )
    elif mutation == 'terminal_gap':
        trajectory = value['terminal_support_context']['raw']['trajectory']
        trajectory.update(
            {
                'last_timestamp_seconds': BINDER_FIXTURES.REQUIRED_END - 0.3,
                'end_gap_seconds': 0.3,
            }
        )
    elif mutation == 'terminal_residual_lidar':
        raw = value['terminal_support_context']['raw']
        record = BINDER_FIXTURES._record('lidar')
        raw['buffers']['lidar'] = BINDER_FIXTURES._buffer('lidar', [record])
        raw['terminal_support_context']['by_topic'] = {'lidar': 1, 'imu': 0, 'image': 0}
        raw['terminal_support_context']['total_count'] = 1
        raw['terminal_support_context']['records'] = [record]
    elif mutation == 'timing_rtf':
        value['timing']['raw']['online_compute_rtf'] = -1.0
    elif mutation == 'safety':
        value['transport']['raw']['scorer_invoked'] = True
    _rewrite_bound(bound, value)
    output = tmp_path / 'invalid.json'
    with pytest.raises(COMPOSITOR.PhaseContractError):
        COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound, output_path=output)
    invalid = json.loads(output.read_text(encoding='utf-8'))
    assert invalid['status'] == 'invalid'
    assert invalid['safety']['ground_truth_content_opened'] is False
    assert output.stat().st_mode & 0o222 == 0


def test_source_hash_drift_missing_timing_and_aliases_fail_closed(tmp_path):
    bound, paths = _bound_fixture(tmp_path)
    callback = paths['callback']
    callback.write_text(callback.read_text(encoding='utf-8') + '\n', encoding='utf-8')
    with pytest.raises(COMPOSITOR.PhaseContractError, match='SOURCE_HASH_DRIFT'):
        COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound)

    bound, paths = _bound_fixture(tmp_path / 'missing')
    paths['timing'].unlink()
    with pytest.raises(COMPOSITOR.PhaseContractError):
        COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound)

    bound, _ = _bound_fixture(tmp_path / 'alias')
    value = _load_bound(bound)
    value['timing']['path'] = value['transport']['path']
    value['timing']['sha256'] = value['transport']['sha256']
    value['sources']['timing_sha256'] = value['transport']['sha256']
    _rewrite_bound(bound, value)
    with pytest.raises(COMPOSITOR.PhaseContractError, match='SOURCE_BINDINGS'):
        COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound)


def test_profile_drift_and_output_overwrite_are_rejected(tmp_path):
    bound, _ = _bound_fixture(tmp_path)
    output = tmp_path / 'composed.json'
    COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound, output_path=output)
    before = output.read_bytes()
    with pytest.raises(COMPOSITOR.PhaseContractError, match='OUTPUT_OVERWRITE'):
        COMPOSITOR.compose(profile_path=PROFILE, bound_path=bound, output_path=output)
    assert output.read_bytes() == before

    drifted = tmp_path / 'profile-drift.yaml'
    drifted.write_bytes(
        PROFILE.read_bytes().replace(
            b'formal_replay_forbidden: true', b'formal_replay_forbidden: false', 1
        )
    )
    with pytest.raises(COMPOSITOR.PhaseContractError, match='PROFILE_DRIFT'):
        COMPOSITOR.compose(profile_path=drifted, bound_path=bound)


def test_v12_compositor_does_not_parse_source_documents_directly():
    text = COMPOSITOR_SCRIPT.read_text(encoding='utf-8')
    normalized = ' '.join(text.split())
    assert '_source_file' in text
    assert '_json(bound_path' in text
    assert '_json(Path(' not in text
    assert 'never parses' in normalized
    assert 'feeder, callback, terminal, or timing source documents' in normalized
