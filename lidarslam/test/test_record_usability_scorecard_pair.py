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

"""Tests for atomic paired usability scorecard observation recording."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def _load(name: str):
    path = SCRIPTS / f'{name}.py'
    spec = importlib.util.spec_from_file_location(f'{name}_test', path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PREPARE = _load('prepare_usability_scorecard_pair')
RECORD = _load('record_usability_scorecard_pair')
CHECK = sys.modules['check_usability_scorecard']


def _public_check(records):
    """Return a schema-valid deterministic result for recorder unit tests."""
    results = []
    for record in records:
        product = record['product']
        requested = product['revision']['value']
        results.append({
            'product_id': product['id'],
            'identity_source': (
                'github.com/rsasaki0109/lidar_slam_ros2'
                if product['id'] == 'lidarslam_ros2'
                else 'github.com/koide3/glim'
            ),
            'revision_kind': product['revision']['kind'],
            'requested_revision': requested,
            'resolved_revision': (
                requested
                if product['revision']['kind'] == 'git-commit'
                else 'c' * 40
            ),
            'documentation_url': product['documentation_root_url'],
            'documentation_http_status': 200,
            'documentation_final_url': product['documentation_root_url'],
        })
    return {
        'performed': True,
        'status': 'PASS',
        'network_reads_performed': True,
        'results': results,
    }


def _prepare(output_dir: Path) -> list[Path]:
    args = [
        '--lidarslam-version', '0.9.1',
        '--lidarslam-revision-kind', 'git-commit',
        '--lidarslam-revision', 'a' * 40,
        '--lidarslam-documentation-url',
        'https://github.com/rsasaki0109/lidar_slam_ros2/tree/'
        + 'a' * 40 + '/docs',
        '--lidarslam-trial-id', 'lidarslam-observed-pair-a',
        '--glim-version', '1.0.0',
        '--glim-revision-kind', 'release-tag',
        '--glim-revision', 'v1.0.0',
        '--glim-documentation-url', 'https://koide3.github.io/glim/',
        '--glim-trial-id', 'glim-observed-pair-a',
        '--cohort-id', 'external-paired-operator-a',
        '--comparison-pair-id', 'paired-jazzy-machine-class-a',
        '--input-id', 'fixed-demo-v1',
        '--clean-start',
        '--ros-distro', 'jazzy',
        '--os-family', 'ubuntu-24.04',
        '--architecture', 'x86_64',
        '--hardware-class', 'eight-core-32gib-x86_64',
        '--machine-fingerprint-sha256', 'b' * 64,
        '--verify-public',
        '--output-dir', str(output_dir),
    ]
    assert PREPARE.main(args, public_verifier=_public_check) == 0
    return sorted(
        path for path in output_dir.glob('*.json')
        if path.name != PREPARE.PREPARATION_RECEIPT_NAME
    )


def _observations(path: Path) -> dict:
    products = {}
    for product_index, product_id in enumerate(CHECK.PRODUCT_IDS):
        tasks = []
        for task_index, contract in enumerate(CHECK.TASK_CONTRACTS):
            measurements = {
                name: task_index + 1
                for name in RECORD.MEASUREMENT_FIELDS
            }
            measurements['wall_time_sec'] = 10.0 + task_index
            measurements['active_operator_time_sec'] = 5.0 + task_index
            tasks.append({
                'task_id': contract['task_id'],
                'exact_commands': [
                    f'public-command-{product_index}-{task_index}'
                ],
                'measurements': measurements,
                'checks': {
                    check_id: True for check_id in contract['checks']
                },
                'undocumented_manual_steps': 0,
                'finding_codes': [],
                'transcript_sha256': str(product_index + 1) * 64,
                'public_url': (
                    f'https://example.test/evidence/{product_id}/'
                    f'{contract["task_id"]}'
                ),
            })
        products[product_id] = {'tasks': tasks}
    value = {'schema_version': 1, 'products': products}
    path.write_text(json.dumps(value), encoding='utf-8')
    return value


def _args(records: list[Path], observations: Path, output: Path) -> list[str]:
    return [
        '--record', str(records[0]),
        '--record', str(records[1]),
        '--observations', str(observations),
        '--output-dir', str(output),
        '--json',
    ]


def test_complete_observations_create_ready_pair_atomically(tmp_path, capsys):
    """One command derives counts/outcomes and emits a checker-ready pair."""
    prepared_dir = tmp_path / 'prepared'
    records = _prepare(prepared_dir)
    capsys.readouterr()
    original_bytes = [path.read_bytes() for path in records]
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations, output)) == 0
    result = json.loads(capsys.readouterr().out)
    completed = [
        json.loads(Path(path).read_text(encoding='utf-8'))
        for path in result['files']
    ]

    assert result['status'] == 'READY'
    assert result['summary']['comparable_tasks'] == 6
    assert result['remote_mutations_performed'] is False
    assert result['automatic_winner_claim_authorized'] is False
    source_receipt = prepared_dir / PREPARE.PREPARATION_RECEIPT_NAME
    retained_receipt = Path(result['preparation_receipt'])
    assert retained_receipt.read_bytes() == source_receipt.read_bytes()
    assert result['preparation_receipt_sha256'] == (
        RECORD.hashlib.sha256(source_receipt.read_bytes()).hexdigest()
    )
    assert {
        path.name: path.read_bytes() for path in records
    } == {
        Path(path).name: Path(path).read_bytes()
        for path in result['prepared_worksheets']
    }
    assert all(path.read_bytes() == before
               for path, before in zip(records, original_bytes))
    assert all(
        task['measurements']['command_count'] == 1
        and task['outcome']['status'] == 'PASS'
        and 'not-recorded' not in task['outcome']['finding_codes']
        for record in completed for task in record['tasks']
    )
    assert CHECK.evaluate_scorecard(completed)['status'] == 'READY'
    assert result['validation_command'].startswith(
        'python3 scripts/check_usability_scorecard.py --record ')
    assert '--preparation-receipt' in result['validation_command']
    assert CHECK.validate_preparation_archive(
        retained_receipt,
        completed,
    )['status'] == 'VALID'


def test_missing_observation_stays_explicit_and_not_ready(tmp_path, capsys):
    """A blank observer value cannot silently become comparable evidence."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    observations_path = tmp_path / 'observations.json'
    observations = _observations(observations_path)
    task = observations['products']['glim']['tasks'][0]
    task['checks']['supported-command-identified'] = None
    observations_path.write_text(json.dumps(observations), encoding='utf-8')
    output = tmp_path / 'recorded'

    args = _args(records, observations_path, output) + ['--require-ready']
    assert RECORD.main(args) == 1
    result = json.loads(capsys.readouterr().out)
    completed = [
        json.loads(Path(path).read_text(encoding='utf-8'))
        for path in result['files']
    ]
    report = CHECK.evaluate_scorecard(completed)

    assert result['status'] == 'PARTIAL'
    assert report['summary']['comparable_tasks'] == 5
    assert 'glim-observation-incomplete' in report['tasks'][0][
        'comparability_blockers']
    assert 'not-recorded' in next(
        record for record in completed if record['product']['id'] == 'glim'
    )['tasks'][0]['outcome']['finding_codes']


def test_existing_output_refuses_every_record(tmp_path, capsys):
    """An existing destination is never replaced by a new observation pair."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'
    output.mkdir()
    marker = output / 'keep.txt'
    marker.write_text('keep\n', encoding='utf-8')

    assert RECORD.main(_args(records, observations, output)) == 2
    assert marker.read_text(encoding='utf-8') == 'keep\n'
    assert list(output.iterdir()) == [marker]
    assert 'refusing to overwrite' in capsys.readouterr().err


def test_pair_drift_is_rejected_before_output(tmp_path, capsys):
    """A changed pair identity cannot be repaired by observation data."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    changed = json.loads(records[0].read_text(encoding='utf-8'))
    changed['environment']['hardware_class'] = 'different-host-class'
    records[0].write_text(json.dumps(changed), encoding='utf-8')
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations, output)) == 2
    assert not output.exists()
    assert 'hardware_class differs' in capsys.readouterr().err


def test_private_command_rejects_the_atomic_pair(tmp_path, capsys):
    """A private path in either transcript command blocks both records."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    observations_path = tmp_path / 'observations.json'
    observations = _observations(observations_path)
    observations['products']['glim']['tasks'][0]['exact_commands'] = [
        'glim /home/operator/private.bag'
    ]
    observations_path.write_text(json.dumps(observations), encoding='utf-8')
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations_path, output)) == 2
    assert not output.exists()
    assert 'private path' in capsys.readouterr().err


def test_noninteractive_stdin_requires_observation_file(
    tmp_path, monkeypatch, capsys,
):
    """Automation cannot hang or invent answers when stdin is not a TTY."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    monkeypatch.setattr(RECORD.sys.stdin, 'isatty', lambda: False)

    args = [
        '--record', str(records[0]),
        '--record', str(records[1]),
        '--output-dir', str(tmp_path / 'recorded'),
    ]
    assert RECORD.main(args) == 2
    assert 'requires a TTY' in capsys.readouterr().err


def test_missing_preparation_receipt_blocks_recording(tmp_path, capsys):
    """Worksheets alone cannot silently become an observed pair."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    (records[0].parent / PREPARE.PREPARATION_RECEIPT_NAME).unlink()
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations, output)) == 2
    assert not output.exists()
    assert 'preparation receipt' in capsys.readouterr().err


def test_reformatted_worksheet_breaks_exact_receipt_hash(tmp_path, capsys):
    """A semantically equal rewrite is still not the prepared byte artifact."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    value = json.loads(records[0].read_text(encoding='utf-8'))
    records[0].write_text(json.dumps(value), encoding='utf-8')
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations, output)) == 2
    assert not output.exists()
    assert 'bytes differ from receipt' in capsys.readouterr().err


def test_tampered_receipt_binding_blocks_recording(tmp_path, capsys):
    """A changed worksheet digest cannot be accepted as preparation proof."""
    records = _prepare(tmp_path / 'prepared')
    capsys.readouterr()
    receipt_path = records[0].parent / PREPARE.PREPARATION_RECEIPT_NAME
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['files'][0]['sha256'] = '0' * 64
    receipt_path.write_text(json.dumps(receipt), encoding='utf-8')
    observations = tmp_path / 'observations.json'
    _observations(observations)
    output = tmp_path / 'recorded'

    assert RECORD.main(_args(records, observations, output)) == 2
    assert not output.exists()
    assert 'file binding differs' in capsys.readouterr().err


def test_recorder_is_retained_in_the_product_release_bundle():
    """The public scorecard docs never point at an omitted release tool."""
    builder = (SCRIPTS / 'build_release_bundle.py').read_text(encoding='utf-8')
    assert "'scripts/prepare_usability_scorecard.py'" in builder
    assert "'scripts/prepare_usability_scorecard_pair.py'" in builder
    assert "'scripts/record_usability_scorecard_pair.py'" in builder
