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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the Docker onboarding machine probe and its safety modes."""

from __future__ import annotations

import builtins
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_docker_onboarding_probe.py'
SPEC = importlib.util.spec_from_file_location(
    'run_docker_onboarding_probe', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def _write_json(path: Path, value: dict[str, object]) -> None:
    path.write_text(json.dumps(value), encoding='utf-8')


def _write_pass_artifacts(run_dir: Path) -> None:
    run_dir.mkdir(parents=True)
    diagnosis_path = run_dir / 'autoware_map_diagnosis.json'
    verify_path = run_dir / 'verify_autoware_map.log'
    manifest_path = run_dir / 'run_manifest.json'
    _write_json(diagnosis_path, {'status': 'success'})
    verify_path.write_text('RESULT: PASS -- verified\n', encoding='utf-8')
    diagnosis_sha256 = PROBE._sha256(diagnosis_path)
    verify_sha256 = PROBE._sha256(verify_path)
    assert diagnosis_sha256 is not None
    assert verify_sha256 is not None
    manifest = {
        'run_id': 'test-run',
        'status': 'succeeded',
        'lifecycle': {
            'stage': 'complete',
            'runner_exit_code': 0,
        },
        'software': {'product_version': '0.9.0'},
        'profile': {'id': 'rko_lio_graph_mid360_preset'},
        'output': {
            'finalized': True,
            'artifact_checksums': [
                {
                    'path': 'autoware_map_diagnosis.json',
                    'sha256': diagnosis_sha256,
                },
                {
                    'path': 'verify_autoware_map.log',
                    'sha256': verify_sha256,
                },
            ],
        },
    }
    _write_json(manifest_path, manifest)
    manifest_sha256 = PROBE._sha256(manifest_path)
    assert manifest_sha256 is not None
    checks = [
        {'id': check_id, 'passed': True, 'observed': 'pass'}
        for check_id in sorted(PROBE.EXPECTED_RECEIPT_CHECKS)
    ]
    _write_json(
        run_dir / 'first_map_validation_receipt.json',
        {
            'schema_version': 1,
            'schema_uri': (
                'https://rsasaki0109.github.io/lidar_slam_ros2/'
                'schemas/first-map-validation-receipt-v1.schema.json'
            ),
            'status': 'PASS',
            'run': {
                'run_id': 'test-run',
                'product_version': '0.9.0',
                'git_commit': None,
                'profile_id': 'rko_lio_graph_mid360_preset',
            },
            'verification': {
                'manifest_status': 'succeeded',
                'diagnosis_status': 'success',
                'autoware_status': 'PASS',
                'manifest_sha256': manifest_sha256,
            },
            'evidence': {
                'manifest': {
                    'filename': 'run_manifest.json',
                    'sha256': manifest_sha256,
                },
                'diagnosis': {
                    'filename': 'autoware_map_diagnosis.json',
                    'available': True,
                    'sha256': diagnosis_sha256,
                },
                'verify_log': {
                    'filename': 'verify_autoware_map.log',
                    'available': True,
                    'sha256': verify_sha256,
                },
            },
            'checks': checks,
            'shareability': {
                'contains_map_geometry': False,
                'contains_private_paths': False,
                'contains_exact_command': False,
                'review_before_sharing': True,
            },
        },
    )


def test_pass_artifacts_are_read_from_final_run(tmp_path):
    """A complete final directory supplies every PASS artifact field."""
    run_dir = tmp_path / 'output' / 'mid360_demo'
    _write_pass_artifacts(run_dir)

    artifact = PROBE._artifact_state(tmp_path)

    assert artifact['run_dir'] == run_dir
    assert artifact['manifest_status'] == 'succeeded'
    assert artifact['diagnosis_status'] == 'success'
    assert artifact['verifier_status'] == 'PASS'
    assert artifact['receipt_status'] == 'PASS'
    assert artifact['receipt_semantic_pass'] is True
    assert artifact['product_version'] == '0.9.0'
    assert artifact['profile_id'] == 'rko_lio_graph_mid360_preset'
    assert len(artifact['manifest_sha256']) == 64
    assert len(artifact['receipt_sha256']) == 64


def test_validated_receipt_is_retained_byte_for_byte_exclusively(tmp_path):
    """The Docker probe copies a reviewed PASS receipt without rewriting."""
    run_dir = tmp_path / 'private' / 'output' / 'mid360_demo'
    _write_pass_artifacts(run_dir)
    artifact = PROBE._artifact_state(tmp_path / 'private')
    output = tmp_path / 'bounded' / 'first-map-validation-receipt.json'

    PROBE._retain_validation_receipt(artifact, output)

    assert output.read_bytes() == artifact['receipt_path'].read_bytes()
    with pytest.raises(PROBE.ProbeError, match='refusing to overwrite'):
        PROBE._retain_validation_receipt(artifact, output)


def test_active_time_is_observed_or_explicitly_unknown(monkeypatch):
    """Active time is separately observed or represented as unknown."""
    answers = iter(('bad', '25', '4.5'))
    monkeypatch.setattr(builtins, 'input', lambda _prompt: next(answers))

    assert PROBE._prompt_active_time(10.0, False) == 4.5
    assert PROBE._prompt_active_time(10.0, True) is None


def test_command_count_is_observed_or_explicitly_unknown(monkeypatch):
    """Command count is separately observed or represented as unknown."""
    answers = iter(('bad', '0', '7'))
    monkeypatch.setattr(builtins, 'input', lambda _prompt: next(answers))

    assert PROBE._prompt_command_count(False) == 7
    assert PROBE._prompt_command_count(True) is None


def test_disk_sampler_reports_peak_delta(tmp_path):
    """Dedicated-host sampling reports only observed allocation growth."""
    sampler = PROBE.DiskSampler(tmp_path)
    sampler.samples = [100, 135, 120]

    assert sampler.peak_delta(100) == 35

    sampler.samples = [99]
    with pytest.raises(PROBE.ProbeError, match='moved below'):
        sampler.peak_delta(100)


def test_malformed_partial_artifacts_fail_closed(tmp_path):
    """Existing but malformed evidence is never promoted to PASS."""
    run_dir = tmp_path / 'output' / 'mid360_demo.partial'
    run_dir.mkdir(parents=True)
    (run_dir / 'run_manifest.json').write_text('{', encoding='utf-8')
    (run_dir / 'first_map_validation_receipt.json').write_text(
        '[]', encoding='utf-8'
    )

    artifact = PROBE._artifact_state(tmp_path)

    assert artifact['run_dir'] == run_dir
    assert artifact['manifest_status'] == 'failed'
    assert artifact['diagnosis_status'] == 'missing'
    assert artifact['verifier_status'] == 'NOT_RUN'
    assert artifact['receipt_status'] == 'FAIL'
    assert artifact['receipt_semantic_pass'] is False
    assert artifact['manifest_sha256'] is not None
    assert artifact['receipt_sha256'] is not None


def test_tampered_pass_artifact_fails_receipt_hash_binding(tmp_path):
    """A PASS-looking log cannot survive a post-receipt hash mismatch."""
    run_dir = tmp_path / 'output' / 'mid360_demo'
    _write_pass_artifacts(run_dir)
    (run_dir / 'verify_autoware_map.log').write_text(
        'RESULT: PASS -- tampered\n', encoding='utf-8'
    )

    artifact = PROBE._artifact_state(tmp_path)

    assert artifact['manifest_status'] == 'succeeded'
    assert artifact['verifier_status'] == 'PASS'
    assert artifact['receipt_status'] == 'FAIL'
    assert artifact['receipt_semantic_pass'] is False


def test_receipt_requires_exactly_the_seven_product_checks(tmp_path):
    """Schema-valid extra checks cannot redefine what PASS means."""
    run_dir = tmp_path / 'output' / 'mid360_demo'
    _write_pass_artifacts(run_dir)
    receipt_path = run_dir / 'first_map_validation_receipt.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['checks'].append({
        'id': 'unexpected_extra_check',
        'passed': True,
        'observed': 'pass',
    })
    _write_json(receipt_path, receipt)

    artifact = PROBE._artifact_state(tmp_path)

    assert artifact['receipt_status'] == 'FAIL'
    assert artifact['receipt_semantic_pass'] is False


def test_missing_receipt_preserves_observed_verifier_status(tmp_path):
    """Receipt failure is not mislabeled as a verifier failure."""
    run_dir = tmp_path / 'output' / 'mid360_demo'
    _write_pass_artifacts(run_dir)
    (run_dir / 'first_map_validation_receipt.json').unlink()

    artifact = PROBE._artifact_state(tmp_path)

    assert artifact['verifier_status'] == 'PASS'
    assert artifact['receipt_status'] == 'NOT_CREATED'
    assert artifact['receipt_semantic_pass'] is False


@pytest.mark.parametrize(
    (
        'archive_bytes',
        'timed_out',
        'receipt_observed',
        'stage',
        'finding',
    ),
    [
        (
            0, False, False,
            'download', 'docker-or-dataset-download-failed',
        ),
        (1, False, False, 'mapping', 'run-manifest-missing'),
        (0, True, False, 'download', 'download-timeout'),
        (1, True, False, 'mapping', 'mapping-timeout'),
        (
            1, True, True,
            'receipt', 'runner-timeout-after-receipt',
        ),
    ],
)
def test_earliest_machine_failure_is_stable(
    archive_bytes,
    timed_out,
    receipt_observed,
    stage,
    finding,
):
    """Machine-observable failures retain stable stages and finding codes."""
    artifact = {
        'manifest_status': 'missing',
        'diagnosis_status': 'missing',
        'verifier_status': 'NOT_RUN',
        'receipt_status': 'NOT_CREATED',
    }

    actual_stage, findings = PROBE._failure_details(
        1,
        artifact,
        archive_bytes,
        timed_out,
        receipt_observed,
    )

    assert actual_stage == stage
    assert findings == [finding]


def test_cli_rejects_mismatched_distro_tag(tmp_path):
    """A Jazzy image cannot be mislabeled as a Humble probe."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-20260810-machine-a',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--record', str(tmp_path / 'record.json'),
            '--allow-privileged-container-host',
        ])

    assert exc_info.value.code == 2


def _candidate_cli_args(tmp_path, *, ref_digest=None, image_digest=None):
    ref_digest = ref_digest or ('a' * 64)
    image_digest = image_digest or ref_digest
    return [
        '--trial-id', 'g0-docker-humble-candidate-12345',
        '--ros-distro', 'humble',
        '--candidate-image-ref',
        'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + ref_digest,
        '--image-digest', 'sha256:' + image_digest,
        '--product-version', '0.9.1',
        '--candidate-image-set-sha256', 'd' * 64,
        '--candidate-evidence-bundle-sha256', 'e' * 64,
        '--candidate-source-pr', '427',
        '--candidate-source-commit', 'b' * 40,
        '--candidate-workflow-run-url',
        'https://github.com/rsasaki0109/lidar_slam_ros2/'
        'actions/runs/12345',
        '--record', str(tmp_path / 'record.json'),
        '--allow-privileged-container-host',
    ]


def test_cli_accepts_tag_free_candidate_and_binds_retained_set(tmp_path):
    """Candidate mode resolves one digest and carries its set identity."""
    args = PROBE._parse_args(_candidate_cli_args(tmp_path))

    assert args.image_tag is None
    assert args.resolved_image_ref == (
        'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'a' * 64
    )
    assert args.candidate_image_evidence == {
        'sha256': 'd' * 64,
        'bundle_sha256': 'e' * 64,
        'source_pr': 427,
        'source_commit': 'b' * 40,
        'product_version': '0.9.1',
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'immutable_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:' + 'a' * 64
        ),
    }


def test_cli_rejects_candidate_ref_digest_mismatch(tmp_path):
    """The retained digest cannot disagree with the executed image ref."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args(_candidate_cli_args(
            tmp_path,
            ref_digest='a' * 64,
            image_digest='b' * 64,
        ))

    assert exc_info.value.code == 2


def test_cli_requires_complete_candidate_set_identity(tmp_path):
    """A bare candidate digest is insufficient trial evidence."""
    args = _candidate_cli_args(tmp_path)
    field_index = args.index('--candidate-image-set-sha256')
    del args[field_index:field_index + 2]

    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args(args)

    assert exc_info.value.code == 2


def test_release_mode_rejects_candidate_evidence_fields(tmp_path):
    """Release and candidate evidence modes cannot be mixed."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-release-mixed',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--candidate-image-set-sha256', 'd' * 64,
            '--record', str(tmp_path / 'record.json'),
            '--allow-privileged-container-host',
        ])

    assert exc_info.value.code == 2


def test_cli_requires_privileged_host_acknowledgement(tmp_path):
    """A privileged nested host always requires explicit acknowledgement."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-20260810-machine-b',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--record', str(tmp_path / 'record.json'),
        ])

    assert exc_info.value.code == 2


def test_cli_requires_explicit_dedicated_filesystem_acknowledgement(tmp_path):
    """Disk sampling cannot be enabled without a dedicated-host boundary."""
    common = [
        '--trial-id', 'g0-docker-humble-20260810-machine-disk',
        '--ros-distro', 'humble',
        '--image-tag',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        '--image-digest', 'sha256:' + ('a' * 64),
        '--record', str(tmp_path / 'record.json'),
        '--allow-privileged-container-host',
    ]
    with pytest.raises(SystemExit) as missing_ack:
        PROBE._parse_args(common + ['--disk-scope', str(tmp_path)])
    assert missing_ack.value.code == 2

    with pytest.raises(SystemExit) as missing_scope:
        PROBE._parse_args(common + ['--acknowledge-dedicated-filesystem'])
    assert missing_scope.value.code == 2

    args = PROBE._parse_args(common + [
        '--disk-scope', str(tmp_path),
        '--acknowledge-dedicated-filesystem',
    ])
    assert args.disk_scope == tmp_path.resolve()


def test_cli_rejects_product_version_mismatch(tmp_path):
    """The public version cannot disagree with the pinned release tag."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-20260810-machine-version',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--product-version', '0.9.1',
            '--record', str(tmp_path / 'record.json'),
            '--allow-privileged-container-host',
        ])

    assert exc_info.value.code == 2


@pytest.mark.parametrize('timeout_value', ['nan', 'inf', '-inf'])
def test_cli_rejects_non_finite_timeout(tmp_path, timeout_value):
    """Non-finite values cannot disable the route deadline."""
    with pytest.raises(SystemExit) as exc_info:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-20260810-machine-timeout',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--record', str(tmp_path / 'record.json'),
            '--timeout-sec', timeout_value,
            '--allow-privileged-container-host',
        ])

    assert exc_info.value.code == 2


def test_cli_rejects_ambiguous_human_measurement_modes(tmp_path):
    """Prompt and explicit-unknown modes cannot be combined."""
    common = [
        '--trial-id', 'g0-docker-humble-20260810-machine-modes',
        '--ros-distro', 'humble',
        '--image-tag',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        '--image-digest', 'sha256:' + ('a' * 64),
        '--record', str(tmp_path / 'record.json'),
        '--allow-privileged-container-host',
    ]
    with pytest.raises(SystemExit) as active_mode:
        PROBE._parse_args(common + [
            '--prompt-active-operator-time',
            '--record-active-time-unknown',
        ])
    assert active_mode.value.code == 2

    with pytest.raises(SystemExit) as command_mode:
        PROBE._parse_args(common + [
            '--prompt-command-count',
            '--record-command-count-unknown',
        ])
    assert command_mode.value.code == 2


def test_combined_human_measurement_prompt_enables_both_observations(tmp_path):
    """The combined switches map to both underlying observation modes."""
    args = PROBE._parse_args([
        '--trial-id', 'g0-docker-humble-comparable',
        '--ros-distro', 'humble',
        '--image-tag',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        '--image-digest', 'sha256:' + ('a' * 64),
        '--record', str(tmp_path / 'record.json'),
        '--allow-privileged-container-host',
        '--prompt-human-measurements',
    ])

    assert args.prompt_active_operator_time is True
    assert args.prompt_command_count is True

    unknown_args = PROBE._parse_args([
        '--trial-id', 'g0-docker-humble-unknown',
        '--ros-distro', 'humble',
        '--image-tag',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        '--image-digest', 'sha256:' + ('a' * 64),
        '--record', str(tmp_path / 'unknown.json'),
        '--allow-privileged-container-host',
        '--record-human-measurements-unknown',
    ])

    assert unknown_args.record_active_time_unknown is True
    assert unknown_args.record_command_count_unknown is True

    with pytest.raises(SystemExit) as mixed_mode:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-comparable',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--record', str(tmp_path / 'record.json'),
            '--allow-privileged-container-host',
            '--prompt-human-measurements',
            '--record-active-time-unknown',
        ])
    assert mixed_mode.value.code == 2

    with pytest.raises(SystemExit) as mixed_unknown_mode:
        PROBE._parse_args([
            '--trial-id', 'g0-docker-humble-unknown',
            '--ros-distro', 'humble',
            '--image-tag',
            'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
            '--image-digest', 'sha256:' + ('a' * 64),
            '--record', str(tmp_path / 'unknown.json'),
            '--allow-privileged-container-host',
            '--record-human-measurements-unknown',
            '--prompt-command-count',
        ])
    assert mixed_unknown_mode.value.code == 2


def test_outer_daemon_rejects_environment_override(monkeypatch):
    """A privileged probe never follows an ambient remote endpoint."""
    monkeypatch.setenv('DOCKER_HOST', 'tcp://example.invalid:2375')

    with pytest.raises(PROBE.ProbeError, match='DOCKER_HOST'):
        PROBE._validate_outer_daemon()


def test_docker_control_command_has_hard_default_timeout(monkeypatch):
    """Every ordinary Docker control call inherits a finite deadline."""
    observed = {}

    def fake_run(command, **kwargs):
        observed['command'] = command
        observed['timeout'] = kwargs.get('timeout')
        return PROBE.subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(PROBE.subprocess, 'run', fake_run)

    PROBE._docker('info')

    assert observed['command'] == [
        'docker',
        '--host',
        PROBE.OUTER_DOCKER_ENDPOINT,
        'info',
    ]
    assert observed['timeout'] == PROBE.DOCKER_CONTROL_TIMEOUT_SEC


def test_missing_observer_image_is_built_once_and_label_verified(monkeypatch):
    """Automatic bootstrap binds the local tag to exact reviewed bytes."""
    dockerfile, _context, recipe = PROBE._observer_recipe()
    image = f'lidarslam-onboarding-trial-host:24.04-{recipe[:12]}'
    image_id = 'sha256:' + ('a' * 64)
    calls = []
    inspections = iter((None, image_id))

    def fake_inspect(value, **kwargs):
        assert value == image
        assert kwargs['expected_ubuntu'] == '24.04'
        assert kwargs['expected_recipe_sha256'] == recipe
        return next(inspections)

    def fake_docker(*arguments, **kwargs):
        calls.append((arguments, kwargs))
        return PROBE.subprocess.CompletedProcess(arguments, 0, '', '')

    monkeypatch.setattr(PROBE, '_inspect_observer_image', fake_inspect)
    monkeypatch.setattr(PROBE, '_docker', fake_docker)
    args = PROBE.argparse.Namespace(
        observer_image=image,
        observer_recipe_sha256=recipe,
        build_observer_image_if_missing=True,
    )

    observed_image, observed_id = PROBE._ensure_observer_image(args, '24.04')

    assert observed_image == image
    assert observed_id == image_id
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command[:2] == ('build', '--pull=false')
    assert f'OBSERVER_RECIPE_SHA256={recipe}' in command
    assert str(dockerfile) in command
    assert kwargs['check'] is False
    assert kwargs['timeout_sec'] == PROBE.OBSERVER_BUILD_TIMEOUT_SEC


def test_existing_observer_image_requires_exact_recipe_labels(monkeypatch):
    """A stale local tag cannot be trusted merely because it exists."""
    image = 'lidarslam-onboarding-trial-host:22.04-' + ('b' * 12)
    payload = [{
        'Id': 'sha256:' + ('c' * 64),
        'Config': {
            'Labels': {
                PROBE.OBSERVER_CONTRACT_LABEL: '1',
                PROBE.OBSERVER_UBUNTU_LABEL: '22.04',
                PROBE.OBSERVER_RECIPE_LABEL: 'd' * 64,
            },
        },
    }]

    monkeypatch.setattr(
        PROBE,
        '_docker',
        lambda *_args, **_kwargs: PROBE.subprocess.CompletedProcess(
            [], 0, json.dumps(payload), ''
        ),
    )

    with pytest.raises(PROBE.ProbeError, match='labels do not match'):
        PROBE._inspect_observer_image(
            image,
            expected_ubuntu='22.04',
            expected_recipe_sha256='e' * 64,
        )


def test_timeout_cleanup_stops_removes_and_confirms_inner_container(
    monkeypatch,
):
    """A route timeout cannot leave its nested product container behind."""
    inner_id = 'c' * 64
    query_results = iter([[inner_id], [inner_id], []])
    control_calls = []

    def fake_ids(_host_name, *, include_stopped):
        control_calls.append(('query', include_stopped))
        return next(query_results)

    def fake_exec(_host_name, *arguments, **_kwargs):
        control_calls.append(arguments)
        return PROBE.subprocess.CompletedProcess(arguments, 0, '', '')

    monkeypatch.setattr(PROBE, '_inner_container_ids', fake_ids)
    monkeypatch.setattr(PROBE, '_docker_exec', fake_exec)

    PROBE._cleanup_timed_out_inner_containers('probe-host')

    assert control_calls == [
        ('query', False),
        ('docker', 'stop', '--time', '10', inner_id),
        ('query', True),
        ('docker', 'rm', '-f', inner_id),
        ('query', True),
    ]


def test_outer_container_validation_accepts_exact_safety_boundary(
    tmp_path,
    monkeypatch,
):
    """The privileged target is bound to its exact image and isolation."""
    container_id = 'a' * 64
    observer_image_id = 'sha256:' + ('b' * 64)
    docker_dir = tmp_path / 'docker'
    trial_dir = tmp_path / 'trial'
    docker_dir.mkdir()
    trial_dir.mkdir()
    value = {
        'Id': container_id,
        'Name': '/probe-host',
        'Image': observer_image_id,
        'Config': {'Image': 'observer:test'},
        'HostConfig': {
            'Privileged': True,
            'NetworkMode': 'bridge',
        },
        'NetworkSettings': {'Networks': {'bridge': {}}},
        'Mounts': [
            {
                'Destination': '/var/lib/docker',
                'Source': str(docker_dir),
                'Type': 'bind',
                'RW': True,
                'Propagation': 'rprivate',
            },
            {
                'Destination': '/trial',
                'Source': str(trial_dir),
                'Type': 'bind',
                'RW': True,
                'Propagation': 'rprivate',
            },
        ],
    }

    def fake_docker(*arguments, **_kwargs):
        assert arguments == ('inspect', 'probe-host')
        return PROBE.subprocess.CompletedProcess(
            ['docker', *arguments], 0, json.dumps([value]), ''
        )

    monkeypatch.setattr(PROBE, '_docker', fake_docker)

    PROBE._validate_outer_container(
        'probe-host',
        container_id,
        'observer:test',
        observer_image_id,
        docker_dir,
        trial_dir,
    )

    value['HostConfig']['Privileged'] = False
    with pytest.raises(PROBE.ProbeError, match='not in the acknowledged mode'):
        PROBE._validate_outer_container(
            'probe-host',
            container_id,
            'observer:test',
            observer_image_id,
            docker_dir,
            trial_dir,
        )


def test_probe_refuses_to_overwrite_bounded_record(tmp_path):
    """Existing evidence is rejected before any Docker operation begins."""
    record = tmp_path / 'record.json'
    record.write_text('{}\n', encoding='utf-8')
    args = PROBE._parse_args([
        '--trial-id', 'g0-docker-humble-20260810-machine-c',
        '--ros-distro', 'humble',
        '--image-tag',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        '--image-digest', 'sha256:' + ('a' * 64),
        '--record', str(record),
        '--allow-privileged-container-host',
    ])

    with pytest.raises(PROBE.ProbeError, match='refusing to overwrite'):
        PROBE.run_probe(args)


def test_bounded_record_writer_is_exclusive(tmp_path):
    """A late writer race cannot replace already-captured evidence."""
    record = tmp_path / 'record.json'
    PROBE._write_json(record, {'status': 'first'})

    with pytest.raises(FileExistsError):
        PROBE._write_json(record, {'status': 'replacement'})

    assert json.loads(record.read_text(encoding='utf-8')) == {
        'status': 'first',
    }
