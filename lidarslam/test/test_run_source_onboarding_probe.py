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

"""Tests for the clean source onboarding machine probe."""

from __future__ import annotations

import argparse
import base64
import builtins
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_source_onboarding_probe.py'
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location(
    'run_source_onboarding_probe', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def _args(tmp_path: Path, *, dry_run: bool = False) -> argparse.Namespace:
    trial = tmp_path / 'trial'
    observer = tmp_path / 'observer-parent'
    trial.mkdir()
    observer.mkdir()
    return argparse.Namespace(
        trial_id='source-jazzy-probe',
        ros_distro='jazzy',
        source_commit='a' * 40,
        product_version='0.9.0',
        trial_root=trial,
        observer_parent=observer,
        disk_scope=tmp_path,
        network_interface=None,
        record=tmp_path / 'bounded-record.json',
        timeout_sec=60.0,
        prompt_active_operator_time=False,
        record_active_time_unknown=True,
        prompt_command_count=False,
        record_command_count_unknown=True,
        acknowledge_disposable_host=True,
        acknowledge_isolated_network=True,
        public_preflight=False,
        dry_run=dry_run,
    )


def _public_route_contents() -> dict[str, str]:
    packages = '\n'.join(PROBE.EXPECTED_SOURCE_PACKAGES)
    return {
        'scripts/source_quickstart.sh': (
            'EXPECTED_SOURCE_PACKAGES=(\n'
            f'{packages}\n'
            ')\n'
            '--packages-select "${EXPECTED_SOURCE_PACKAGES[@]}"\n'
            '[source-package-inventory-mismatch]\n'
            'install_source_dependencies.sh --repo-only\n'
            '-DBUILD_TESTING=OFF\n'
            'set +u\nsource "${ROS_SETUP}"\nset -u\n'
            'set +u\nsource "${INSTALL_SETUP}"\nset -u\n'
            'lidarslam-map demo\n'
        ),
        'scripts/install_source_dependencies.sh': (
            '--repo-only\n'
            'rosdep install \\\n'
            '  --from-paths "${DEPENDENCY_ROOT}" \\\n'
            '  --ignore-src\n'
        ),
        'docs/getting-started.md': (
            'bash scripts/source_quickstart.sh\n'
            '6 ROS packages with BUILD_TESTING=OFF\n'
            'without changing your shell\n'
        ),
        'VERSION': '0.9.0\n',
    }


def test_dry_run_never_queries_public_source_or_writes(
    monkeypatch,
    tmp_path,
    capsys,
):
    args = _args(tmp_path, dry_run=True)
    monkeypatch.setattr(PROBE, '_validate_host', lambda _distro: None)
    monkeypatch.setattr(PROBE, '_network_interface', lambda _value: 'eth0')
    monkeypatch.setattr(
        PROBE,
        '_preflight_public_source',
        lambda *_args: pytest.fail('dry-run queried GitHub'),
    )
    before = sorted(path.relative_to(tmp_path) for path in tmp_path.rglob('*'))

    record, observer = PROBE.run_probe(args)

    assert record == {}
    assert observer is None
    assert not args.record.exists()
    after = sorted(path.relative_to(tmp_path) for path in tmp_path.rglob('*'))
    assert after == before
    plan = json.loads(capsys.readouterr().out)
    assert plan['network_or_writes_performed'] is False
    assert plan['source_commit'] == 'a' * 40
    assert plan['stages'][0] == 'verify_public_source_identity'
    assert 'run_source_quickstart_headless' in plan['stages']
    assert 'capture_observed_command_count' in plan['stages']


def test_publicly_missing_commit_writes_valid_bounded_fail(
    monkeypatch,
    tmp_path,
):
    args = _args(tmp_path)
    monkeypatch.setattr(PROBE, '_validate_host', lambda _distro: None)
    monkeypatch.setattr(PROBE, '_network_interface', lambda _value: 'eth0')

    def unavailable(*_args):
        raise PROBE.RouteUnavailable(
            'source-candidate-not-published',
            'exact commit is absent',
        )

    monkeypatch.setattr(PROBE, '_preflight_public_source', unavailable)

    record, observer = PROBE.run_probe(args)

    assert observer is None
    assert record['outcome']['status'] == 'FAIL'
    assert record['outcome']['failure_stage'] == 'preflight'
    assert record['outcome']['runner_exit_code'] is None
    assert record['outcome']['finding_codes'] == [
        'source-candidate-not-published'
    ]
    assert record['measurements']['command_count'] is None
    assert record['input']['download_bytes'] is None
    assert args.record.is_file()
    assert json.loads(args.record.read_text(encoding='utf-8')) == record
    report = PROBE.evaluate_trial(record)
    assert report['comparable'] is False
    assert 'outcome_failed' in report['comparability_blockers']


def test_source_probe_retains_exact_validated_receipt(tmp_path):
    """The source probe preserves exact receipt bytes and never overwrites."""
    private = tmp_path / 'private-receipt.json'
    payload = b'{"status":"PASS"}\n'
    private.write_bytes(payload)
    output = tmp_path / 'bounded-receipt.json'
    artifact = {
        'receipt_semantic_pass': True,
        'receipt_path': private,
        'receipt_sha256': PROBE.hashlib.sha256(payload).hexdigest(),
    }

    PROBE._retain_validation_receipt(artifact, output)

    assert output.read_bytes() == payload
    with pytest.raises(PROBE.ProbeError, match='refusing to overwrite'):
        PROBE._retain_validation_receipt(artifact, output)


def test_content_decoder_accepts_github_base64_line_wrapping(monkeypatch):
    encoded = base64.b64encode(b'bash scripts/source_quickstart.sh\n').decode()
    wrapped = encoded[:12] + '\n' + encoded[12:] + '\n'
    monkeypatch.setattr(
        PROBE,
        '_request_json',
        lambda _url: (200, {'type': 'file', 'content': wrapped}),
    )

    assert PROBE._content_at('a' * 40, 'scripts/source_quickstart.sh') == (
        'bash scripts/source_quickstart.sh\n'
    )


def test_public_preflight_token_is_scoped_to_github_api(monkeypatch):
    """The optional token is sent only to GitHub API requests."""
    requests = []

    class FakeResponse:
        status = 200

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return b'{}'

    def fake_urlopen(request, timeout):
        assert timeout == 20
        requests.append(request)
        return FakeResponse()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(PROBE.urllib.request, 'urlopen', fake_urlopen)

    PROBE._request_json(f'{PROBE.GITHUB_API}/commits/' + 'a' * 40)
    PROBE._request_json(
        'https://raw.githubusercontent.com/owner/repo/main/file')

    assert requests[0].get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert requests[1].get_header('Authorization') is None


def test_public_source_preflight_requires_commit_route_and_version(monkeypatch):
    contents = _public_route_contents()

    def request(url: str):
        if '/commits/' in url:
            return 200, {'sha': 'a' * 40}
        relative = url.split('/contents/', 1)[1].split('?ref=', 1)[0]
        payload = base64.b64encode(contents[relative].encode()).decode()
        return 200, {'type': 'file', 'content': payload}

    monkeypatch.setattr(PROBE, '_request_json', request)
    details = PROBE._preflight_public_source('a' * 40, '0.9.0')

    assert details['source_packages'] == list(PROBE.EXPECTED_SOURCE_PACKAGES)
    assert details['product_version'] == '0.9.0'

    contents['VERSION'] = '0.8.0\n'
    with pytest.raises(PROBE.RouteUnavailable) as error:
        PROBE._preflight_public_source('a' * 40, '0.9.0')
    assert error.value.code == 'source-version-mismatch'


@pytest.mark.parametrize('status', (404, 422))
def test_public_source_preflight_classifies_unpublished_commit_statuses(
    monkeypatch,
    status,
):
    """Both GitHub unknown-ref responses mean the exact tip is not public."""
    monkeypatch.setattr(
        PROBE,
        '_request_json',
        lambda _url: (status, None),
    )

    with pytest.raises(PROBE.RouteUnavailable) as error:
        PROBE._preflight_public_source('a' * 40, '0.9.0')

    assert error.value.code == 'source-candidate-not-published'


def test_content_endpoint_422_remains_an_observer_error(monkeypatch):
    """A bad content request must not masquerade as an absent route file."""
    monkeypatch.setattr(
        PROBE,
        '_request_json',
        lambda _url: (422, None),
    )

    with pytest.raises(PROBE.ProbeError, match='content inspection.*422'):
        PROBE._content_at('a' * 40, 'scripts/source_quickstart.sh')


def test_public_source_preflight_rejects_package_or_fast_route_drift(
    monkeypatch,
):
    contents = _public_route_contents()

    def request(url: str):
        if '/commits/' in url:
            return 200, {'sha': 'a' * 40}
        relative = url.split('/contents/', 1)[1].split('?ref=', 1)[0]
        payload = base64.b64encode(contents[relative].encode()).decode()
        return 200, {'type': 'file', 'content': payload}

    monkeypatch.setattr(PROBE, '_request_json', request)
    contents['scripts/source_quickstart.sh'] = contents[
        'scripts/source_quickstart.sh'
    ].replace('scanmatcher\n)', 'research_only_package\n)')
    with pytest.raises(PROBE.RouteUnavailable) as package_error:
        PROBE._preflight_public_source('a' * 40, '0.9.0')
    assert package_error.value.code == 'source-route-contract-missing'

    contents.update(_public_route_contents())
    contents['docs/getting-started.md'] = contents[
        'docs/getting-started.md'
    ].replace('BUILD_TESTING=OFF', 'tests enabled')
    with pytest.raises(PROBE.RouteUnavailable) as route_error:
        PROBE._preflight_public_source('a' * 40, '0.9.0')
    assert route_error.value.code == 'source-route-contract-missing'


def test_public_preflight_report_is_read_only_and_machine_decidable(
    monkeypatch,
):
    assert PROBE.sys.dont_write_bytecode is True
    monkeypatch.setattr(
        PROBE,
        '_preflight_public_source',
        lambda commit, version: {
            'commit': commit,
            'product_version': version,
            'source_packages': list(PROBE.EXPECTED_SOURCE_PACKAGES),
        },
    )

    ready = PROBE._public_preflight_report('a' * 40, '0.9.0')

    assert ready['status'] == 'READY'
    assert ready['writes_performed'] is False
    assert ready['finding_codes'] == []

    def unavailable(*_args):
        raise PROBE.RouteUnavailable(
            'source-candidate-not-published',
            'exact commit is absent',
        )

    monkeypatch.setattr(PROBE, '_preflight_public_source', unavailable)
    not_ready = PROBE._public_preflight_report('b' * 40, '0.9.0')
    assert not_ready['status'] == 'NOT_READY'
    assert not_ready['writes_performed'] is False
    assert not_ready['finding_codes'] == ['source-candidate-not-published']


def test_private_route_fetches_only_the_exact_public_commit(tmp_path):
    args = _args(tmp_path)
    observer = tmp_path / 'private-observer'
    observer.mkdir()

    route = PROBE._route_script(args, observer)

    text = route.read_text(encoding='utf-8')
    assert 'git clone' not in text
    assert 'init.defaultBranch=detached init' in text
    assert f'remote add origin {PROBE.REPO_URL}' in text
    assert f'fetch --depth=1 origin {args.source_commit}' in text
    assert 'checkout --detach FETCH_HEAD' in text
    assert 'submodule update --init --recursive' in text
    assert 'bash scripts/source_quickstart.sh' in text
    assert '--workspace' in text
    assert '--ros-distro jazzy --viewer none' in text
    assert str(ROOT) not in text
    assert route.stat().st_mode & 0o777 == 0o700


def test_path_contract_rejects_dirty_overlap_and_existing_record(tmp_path):
    args = _args(tmp_path)
    (args.trial_root / 'old-output').mkdir()
    with pytest.raises(PROBE.ProbeError, match='not empty'):
        PROBE._validate_paths(args)

    (args.trial_root / 'old-output').rmdir()
    args.observer_parent = tmp_path
    with pytest.raises(PROBE.ProbeError, match='must not overlap'):
        PROBE._validate_paths(args)

    args.observer_parent = tmp_path / 'observer-parent'
    args.record.write_text('{}\n', encoding='utf-8')
    with pytest.raises(PROBE.ProbeError, match='overwrite'):
        PROBE._validate_paths(args)

    args.record.unlink()
    outside = tmp_path / 'unexpected-record.json'
    args.record.symlink_to(outside)
    with pytest.raises(PROBE.ProbeError, match='overwrite'):
        PROBE._validate_paths(args)
    assert not outside.exists()


def test_disk_sampler_reports_peak_delta_and_propagates_errors(tmp_path):
    values = iter((100, 130, 170, 150))

    def reader(_scope: Path) -> int:
        try:
            return next(values)
        except StopIteration:
            return 150

    sampler = PROBE._DiskSampler(tmp_path, reader)
    baseline = sampler.start()
    time.sleep(PROBE.SAMPLE_INTERVAL_SEC * 3.5)
    sampler.stop()

    assert baseline == 100
    assert sampler.peak_delta(baseline) == 70
    assert len(sampler.samples) >= 3

    def broken(_scope: Path) -> int:
        if getattr(broken, 'called', False):
            raise RuntimeError('sample failed')
        broken.called = True
        return 10

    failed = PROBE._DiskSampler(tmp_path, broken)
    failed.start()
    time.sleep(PROBE.SAMPLE_INTERVAL_SEC * 1.5)
    with pytest.raises(PROBE.ProbeError, match='sample failed'):
        failed.stop()


def test_active_time_is_observed_or_explicitly_unknown(monkeypatch):
    answers = iter(('bad', '25', '4.5'))
    monkeypatch.setattr(builtins, 'input', lambda _prompt: next(answers))

    assert PROBE._prompt_active_time(10.0, False) == 4.5
    assert PROBE._prompt_active_time(10.0, True) is None


def test_command_count_is_observed_or_explicitly_unknown(monkeypatch):
    answers = iter(('bad', '0', '7'))
    monkeypatch.setattr(builtins, 'input', lambda _prompt: next(answers))

    assert PROBE._prompt_command_count(False) == 7
    assert PROBE._prompt_command_count(True) is None


@pytest.mark.parametrize(
    ('setup', 'archive_bytes', 'timed_out', 'stage', 'finding'),
    (
        (False, 0, False, 'install', 'source-clone-failed'),
        (True, 0, False, 'install', 'source-install-failed'),
        (True, 0, True, 'install', 'source-install-timeout'),
    ),
)
def test_source_failure_classification_is_stable(
    tmp_path,
    setup,
    archive_bytes,
    timed_out,
    stage,
    finding,
):
    args = _args(tmp_path)
    if setup:
        checkout = args.trial_root / 'src' / 'lidar_slam_ros2'
        checkout.mkdir(parents=True)
    artifact = {
        'manifest_status': 'missing',
        'diagnosis_status': 'missing',
        'verifier_status': 'NOT_RUN',
        'receipt_status': 'NOT_CREATED',
    }

    actual_stage, findings = PROBE._failure_details(
        args, artifact, archive_bytes, timed_out
    )

    assert actual_stage == stage
    assert findings == [finding]


def test_source_package_inventory_failure_survives_private_log_boundary(
    tmp_path,
):
    args = _args(tmp_path)
    checkout = args.trial_root / 'src' / 'lidar_slam_ros2'
    checkout.mkdir(parents=True)
    route_log = tmp_path / 'source-route.log'
    route_log.write_text(
        'earlier private output\n'
        'error: [source-package-inventory-mismatch] expected six\n',
        encoding='utf-8',
    )
    artifact = {
        'manifest_status': 'missing',
        'diagnosis_status': 'missing',
        'verifier_status': 'NOT_RUN',
        'receipt_status': 'NOT_CREATED',
    }

    stage, findings = PROBE._failure_details(
        args,
        artifact,
        archive_bytes=0,
        timed_out=False,
        route_log=route_log,
    )

    assert stage == 'install'
    assert findings == ['source-package-inventory-mismatch']


def test_cli_rejects_ambiguous_active_time_and_bad_commit(tmp_path):
    common = [
        '--trial-id', 'source-jazzy-probe',
        '--ros-distro', 'jazzy',
        '--source-commit', 'A' * 40,
        '--product-version', '0.9.0',
        '--trial-root', str(tmp_path),
        '--observer-parent', str(tmp_path),
        '--disk-scope', str(tmp_path),
        '--record', str(tmp_path / 'record.json'),
        '--dry-run',
    ]
    with pytest.raises(SystemExit) as bad_commit:
        PROBE._parse_args(common)
    assert bad_commit.value.code == 2

    common[common.index('A' * 40)] = 'a' * 40
    common.extend([
        '--prompt-active-operator-time',
        '--record-active-time-unknown',
    ])
    with pytest.raises(SystemExit) as active_mode:
        PROBE._parse_args(common)
    assert active_mode.value.code == 2

    common = [item for item in common if item not in {
        '--prompt-active-operator-time',
        '--record-active-time-unknown',
    }]
    common.extend([
        '--prompt-command-count',
        '--record-command-count-unknown',
    ])
    with pytest.raises(SystemExit) as command_mode:
        PROBE._parse_args(common)
    assert command_mode.value.code == 2


def test_combined_human_measurement_prompt_enables_both_observations(tmp_path):
    args = PROBE._parse_args([
        '--trial-id', 'source-jazzy-comparable',
        '--ros-distro', 'jazzy',
        '--source-commit', 'a' * 40,
        '--product-version', '0.9.0',
        '--trial-root', str(tmp_path),
        '--observer-parent', str(tmp_path),
        '--disk-scope', str(tmp_path),
        '--record', str(tmp_path / 'record.json'),
        '--prompt-human-measurements',
        '--dry-run',
    ])

    assert args.prompt_active_operator_time is True
    assert args.prompt_command_count is True

    unknown_args = PROBE._parse_args([
        '--trial-id', 'source-jazzy-unknown',
        '--ros-distro', 'jazzy',
        '--source-commit', 'a' * 40,
        '--product-version', '0.9.0',
        '--trial-root', str(tmp_path),
        '--observer-parent', str(tmp_path),
        '--disk-scope', str(tmp_path),
        '--record', str(tmp_path / 'unknown.json'),
        '--record-human-measurements-unknown',
        '--dry-run',
    ])

    assert unknown_args.record_active_time_unknown is True
    assert unknown_args.record_command_count_unknown is True

    with pytest.raises(SystemExit) as mixed_mode:
        PROBE._parse_args([
            '--trial-id', 'source-jazzy-comparable',
            '--ros-distro', 'jazzy',
            '--source-commit', 'a' * 40,
            '--product-version', '0.9.0',
            '--trial-root', str(tmp_path),
            '--observer-parent', str(tmp_path),
            '--disk-scope', str(tmp_path),
            '--record', str(tmp_path / 'record.json'),
            '--prompt-human-measurements',
            '--record-command-count-unknown',
            '--dry-run',
        ])
    assert mixed_mode.value.code == 2

    with pytest.raises(SystemExit) as mixed_unknown_mode:
        PROBE._parse_args([
            '--trial-id', 'source-jazzy-unknown',
            '--ros-distro', 'jazzy',
            '--source-commit', 'a' * 40,
            '--product-version', '0.9.0',
            '--trial-root', str(tmp_path),
            '--observer-parent', str(tmp_path),
            '--disk-scope', str(tmp_path),
            '--record', str(tmp_path / 'unknown.json'),
            '--record-human-measurements-unknown',
            '--prompt-command-count',
            '--dry-run',
        ])
    assert mixed_unknown_mode.value.code == 2


def test_public_preflight_cli_needs_only_immutable_source_identity():
    args = PROBE._parse_args([
        '--public-preflight',
        '--source-commit',
        'a' * 40,
        '--product-version',
        '0.9.0',
    ])

    assert args.public_preflight is True
    assert args.trial_root is None
    assert args.ros_distro is None

    with pytest.raises(SystemExit) as execution_mix:
        PROBE._parse_args([
            '--public-preflight',
            '--source-commit',
            'a' * 40,
            '--product-version',
            '0.9.0',
            '--ros-distro',
            'jazzy',
        ])
    assert execution_mix.value.code == 2


def test_real_cli_dry_run_is_read_only(tmp_path):
    os_version = PROBE._os_version()
    distro = {'22.04': 'humble', '24.04': 'jazzy'}.get(os_version)
    if distro is None or not (Path('/opt/ros') / distro / 'setup.bash').is_file():
        pytest.skip('host is not a supported source-probe environment')
    trial = tmp_path / 'trial'
    observer = tmp_path / 'observer'
    trial.mkdir()
    observer.mkdir()
    record = tmp_path / 'record.json'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--trial-id',
            f'source-{distro}-dry-run',
            '--ros-distro',
            distro,
            '--source-commit',
            'a' * 40,
            '--product-version',
            '0.9.0',
            '--trial-root',
            str(trial),
            '--observer-parent',
            str(observer),
            '--disk-scope',
            str(tmp_path),
            '--record',
            str(record),
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    assert plan['network_or_writes_performed'] is False
    assert not record.exists()
    assert list(trial.iterdir()) == []
    assert list(observer.iterdir()) == []
