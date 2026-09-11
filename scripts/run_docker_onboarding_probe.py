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

"""Run a privacy-bounded Docker onboarding machine probe.

The probe uses a fresh Docker daemon inside a disposable Ubuntu container. It
does not mount the host Docker socket or share project image/data caches. This
is non-comparable when run on a shared host because that host does not provide
the dedicated filesystem required for peak-disk measurement. A dedicated VM
may opt into host-filesystem sampling with an explicit acknowledgement. The
script can retain separately observed human active time and command count, but
it never infers either value from its own harness commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO

import jsonschema


SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-trial-v1.schema.json'
)
IMAGE_TAG_RE = re.compile(
    r'^ghcr\.io/rsasaki0109/lidar_slam_ros2:'
    r'v(?P<version>[0-9]+\.[0-9]+\.[0-9]+'
    r'(?:[-+][A-Za-z0-9.-]+)?)-(?P<distro>humble|jazzy)$'
)
CANDIDATE_IMAGE_REF_RE = re.compile(
    r'^ghcr\.io/rsasaki0109/lidar_slam_ros2@'
    r'(?P<digest>sha256:[0-9a-f]{64})$'
)
DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
WORKFLOW_RUN_URL_RE = re.compile(
    r'^https://github\.com/rsasaki0109/lidar_slam_ros2/'
    r'actions/runs/[1-9][0-9]*$'
)
TRIAL_ID_RE = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')
VERSION_RE = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$'
)
OS_VERSION = {'humble': '22.04', 'jazzy': '24.04'}
REPO_ROOT = Path(__file__).resolve().parents[1]
RECEIPT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'first-map-validation-receipt-v1.schema.json'
)
DOCKER_CONTROL_TIMEOUT_SEC = 30.0
OBSERVER_BUILD_TIMEOUT_SEC = 900.0
POST_RECEIPT_GRACE_SEC = 60.0
DISK_SAMPLE_INTERVAL_SEC = 0.25
OUTER_DOCKER_ENDPOINT = 'unix:///var/run/docker.sock'
OBSERVER_CONTRACT_VERSION = '1'
OBSERVER_CONTRACT_LABEL = (
    'io.github.rsasaki0109.lidarslam.observer-contract'
)
OBSERVER_UBUNTU_LABEL = 'io.github.rsasaki0109.lidarslam.observer-ubuntu'
OBSERVER_RECIPE_LABEL = (
    'io.github.rsasaki0109.lidarslam.observer-recipe-sha256'
)
OBSERVER_IMAGE_RE = re.compile(
    r'^lidarslam-onboarding-trial-host:'
    r'(?:22\.04|24\.04)(?:-[0-9a-f]{12})?$'
)
EXPECTED_RECEIPT_CHECKS = {
    'manifest_succeeded',
    'lifecycle_complete',
    'runner_exit_zero',
    'diagnosis_success',
    'autoware_verification_pass',
    'diagnosis_bound_to_manifest',
    'verify_log_bound_to_manifest',
}
ARCHIVE_RELATIVE = Path(
    'data/driving_slam_mid360/archives/'
    'rosbag2_2024_04_16-14_17_01.zip'
)


class ProbeError(RuntimeError):
    """The observer harness could not produce a trustworthy record."""


def _prompt_active_time(wall_time: float, unknown: bool) -> float | None:
    """Capture a separately observed paused stopwatch without guessing."""
    if unknown:
        return None
    while True:
        try:
            value = input(
                'Observed active operator seconds '
                '(paused stopwatch; blank records unknown): '
            ).strip()
        except EOFError:
            return None
        if not value:
            return None
        try:
            parsed = float(value)
        except ValueError:
            print('Enter a finite number or leave blank.', file=sys.stderr)
            continue
        if not math.isfinite(parsed) or parsed < 0 or parsed > wall_time:
            print(
                f'Enter a value from 0 through {wall_time:.3f}.',
                file=sys.stderr,
            )
            continue
        return round(parsed, 3)


def _prompt_command_count(unknown: bool) -> int | None:
    """Capture human-submitted command count without counting the harness."""
    if unknown:
        return None
    while True:
        try:
            value = input(
                'Observed human-submitted command count '
                '(blank records unknown): '
            ).strip()
        except EOFError:
            return None
        if not value:
            return None
        try:
            parsed = int(value)
        except ValueError:
            print('Enter a positive integer or leave blank.', file=sys.stderr)
            continue
        if parsed < 1:
            print('Enter a positive integer or leave blank.', file=sys.stderr)
            continue
        return parsed


class DiskSampler:
    """Sample allocated bytes on an explicitly dedicated host filesystem."""

    def __init__(self, scope: Path) -> None:
        """Create a sampler for one already-validated filesystem scope."""
        self.scope = scope
        self.samples: list[int] = []
        self.error: BaseException | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _used_bytes(self) -> int:
        usage = shutil.disk_usage(self.scope)
        return usage.total - usage.free

    def start(self) -> int:
        """Capture the baseline and start bounded background sampling."""
        if self._thread is not None:
            raise ProbeError('disk sampler was started twice')
        baseline = self._used_bytes()
        self.samples.append(baseline)
        self._thread = threading.Thread(
            target=self._sample,
            daemon=True,
        )
        self._thread.start()
        return baseline

    def _sample(self) -> None:
        try:
            while not self._stop.wait(DISK_SAMPLE_INTERVAL_SEC):
                self.samples.append(self._used_bytes())
        except BaseException as exc:  # propagated on the controlling thread
            self.error = exc
            self._stop.set()

    def stop(self) -> None:
        """Stop sampling and retain one final observation."""
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise ProbeError('disk sampler did not stop')
        self._thread = None
        if self.error is not None:
            raise ProbeError(f'disk sampler failed: {self.error}')
        self.samples.append(self._used_bytes())

    def peak_delta(self, baseline: int) -> int:
        """Return the largest allocated-byte increase from the baseline."""
        if not self.samples:
            raise ProbeError('disk sampler produced no observations')
        peak = max(self.samples) - baseline
        if peak < 0:
            raise ProbeError('disk allocation moved below the baseline')
        return peak


def _run(
    command: list[str],
    *,
    check: bool = True,
    timeout_sec: float = DOCKER_CONTROL_TIMEOUT_SEC,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=check,
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )


def _docker(
    *arguments: str,
    check: bool = True,
    timeout_sec: float = DOCKER_CONTROL_TIMEOUT_SEC,
) -> subprocess.CompletedProcess[str]:
    return _run(
        ['docker', '--host', OUTER_DOCKER_ENDPOINT, *arguments],
        check=check,
        timeout_sec=timeout_sec,
    )


def _docker_exec(
    host_name: str,
    *arguments: str,
    check: bool = True,
    timeout_sec: float = DOCKER_CONTROL_TIMEOUT_SEC,
) -> subprocess.CompletedProcess[str]:
    return _docker(
        'exec',
        host_name,
        *arguments,
        check=check,
        timeout_sec=timeout_sec,
    )


def _observer_recipe() -> tuple[Path, Path, str]:
    """Return the reviewed observer recipe and its exact source hash."""
    dockerfile = REPO_ROOT / 'docker' / 'onboarding-trial-host.Dockerfile'
    context = REPO_ROOT / 'docker'
    try:
        if dockerfile.is_symlink() or not dockerfile.is_file():
            raise ProbeError(
                'observer Dockerfile must be one regular non-symlink file'
            )
        if context.is_symlink() or not context.is_dir():
            raise ProbeError(
                'observer build context must be one real directory'
            )
        recipe_sha256 = hashlib.sha256(dockerfile.read_bytes()).hexdigest()
    except OSError as exc:
        raise ProbeError(
            f'cannot inspect observer build recipe: {exc}'
        ) from exc
    return dockerfile, context, recipe_sha256


def _inspect_observer_image(
    image: str,
    *,
    expected_ubuntu: str,
    expected_recipe_sha256: str | None,
) -> str | None:
    """Return one validated local image ID, or None when it is absent."""
    result = _docker('image', 'inspect', image, check=False)
    if result.returncode != 0:
        return None
    try:
        image_values = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise ProbeError('cannot parse observer image inspection') from exc
    if (
        not isinstance(image_values, list)
        or len(image_values) != 1
        or not isinstance(image_values[0], dict)
    ):
        raise ProbeError('observer image inspection is not singular')
    observer_image_id = image_values[0].get('Id')
    if not isinstance(observer_image_id, str) or not re.fullmatch(
        r'sha256:[0-9a-f]{64}', observer_image_id
    ):
        raise ProbeError('observer image has no immutable local ID')
    if expected_recipe_sha256 is not None:
        labels = image_values[0].get('Config', {}).get('Labels')
        expected_labels = {
            OBSERVER_CONTRACT_LABEL: OBSERVER_CONTRACT_VERSION,
            OBSERVER_UBUNTU_LABEL: expected_ubuntu,
            OBSERVER_RECIPE_LABEL: expected_recipe_sha256,
        }
        if not isinstance(labels, dict) or any(
            labels.get(name) != value
            for name, value in expected_labels.items()
        ):
            raise ProbeError(
                'observer image labels do not match the reviewed recipe'
            )
    return observer_image_id


def _ensure_observer_image(
    args: argparse.Namespace,
    os_version: str,
) -> tuple[str, str]:
    """Inspect or safely bootstrap the exact local observer image."""
    observer_image = getattr(
        args,
        'observer_image',
        None,
    ) or f'lidarslam-onboarding-trial-host:{os_version}'
    expected_recipe = getattr(args, 'observer_recipe_sha256', None)
    image_id = _inspect_observer_image(
        observer_image,
        expected_ubuntu=os_version,
        expected_recipe_sha256=expected_recipe,
    )
    if image_id is not None:
        return observer_image, image_id
    if not getattr(args, 'build_observer_image_if_missing', False):
        raise ProbeError(
            f'observer image missing: build {observer_image} from '
            'docker/onboarding-trial-host.Dockerfile'
        )

    dockerfile, context, actual_recipe = _observer_recipe()
    if expected_recipe is not None and expected_recipe != actual_recipe:
        raise ProbeError(
            'requested observer recipe hash does not match the local '
            'Dockerfile'
        )
    expected_recipe = actual_recipe
    print(
        f'Preparing reviewed Docker observer image: {observer_image}',
        file=sys.stderr,
    )
    build = _docker(
        'build',
        '--pull=false',
        '--build-arg',
        f'UBUNTU_VERSION={os_version}',
        '--build-arg',
        f'OBSERVER_RECIPE_SHA256={expected_recipe}',
        '--file',
        str(dockerfile),
        '--tag',
        observer_image,
        str(context),
        check=False,
        timeout_sec=OBSERVER_BUILD_TIMEOUT_SEC,
    )
    if build.returncode != 0:
        detail = build.stderr.strip() or build.stdout.strip()
        if detail:
            detail = detail.splitlines()[-1][:300]
        else:
            detail = f'exit {build.returncode}'
        raise ProbeError(f'observer image build failed: {detail}')
    image_id = _inspect_observer_image(
        observer_image,
        expected_ubuntu=os_version,
        expected_recipe_sha256=expected_recipe,
    )
    if image_id is None:
        raise ProbeError('observer image build completed without its tag')
    return observer_image, image_id


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError:
        return None
    return digest.hexdigest()


def _verify_log_status(path: Path | None) -> str:
    if path is None or not path.is_file():
        return 'missing'
    try:
        value = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return 'missing'
    match = re.search(
        r'^RESULT:\s*(PASS|FAIL)(?:\s+--[^\r\n]*)?\s*$',
        value,
        re.MULTILINE,
    )
    return match.group(1) if match else 'unknown'


def _manifest_artifact_hashes(
    manifest: dict[str, Any] | None,
) -> dict[str, str]:
    if not manifest:
        return {}
    output = manifest.get('output')
    if not isinstance(output, dict):
        return {}
    identities = output.get('artifact_checksums')
    if not isinstance(identities, list):
        return {}
    result: dict[str, str] = {}
    for identity in identities:
        if not isinstance(identity, dict):
            continue
        path = identity.get('path')
        digest = identity.get('sha256')
        if isinstance(path, str) and isinstance(digest, str):
            result[path] = digest
    return result


def _receipt_semantically_passes(
    receipt: dict[str, Any] | None,
    manifest: dict[str, Any] | None,
    diagnosis: dict[str, Any] | None,
    manifest_sha256: str | None,
    diagnosis_sha256: str | None,
    verify_log_sha256: str | None,
    verify_log_status: str,
) -> bool:
    schema = _load_json(RECEIPT_SCHEMA)
    if not all(isinstance(value, dict) for value in (
        receipt,
        manifest,
        diagnosis,
        schema,
    )):
        return False
    assert receipt is not None
    assert manifest is not None
    assert diagnosis is not None
    assert schema is not None
    if not jsonschema.Draft7Validator(schema).is_valid(receipt):
        return False

    lifecycle = manifest.get('lifecycle')
    software = manifest.get('software')
    profile = manifest.get('profile')
    output = manifest.get('output')
    verification = receipt.get('verification')
    evidence = receipt.get('evidence')
    run = receipt.get('run')
    checks = receipt.get('checks')
    if not all(isinstance(value, dict) for value in (
        lifecycle,
        software,
        profile,
        output,
        verification,
        evidence,
        run,
    )) or not isinstance(checks, list):
        return False

    check_ids = {
        item.get('id')
        for item in checks
        if isinstance(item, dict) and item.get('passed') is True
    }
    if (
        len(checks) != len(EXPECTED_RECEIPT_CHECKS)
        or check_ids != EXPECTED_RECEIPT_CHECKS
    ):
        return False
    if not all(
        isinstance(item, dict) and item.get('passed') is True
        for item in checks
    ):
        return False

    manifest_evidence = evidence.get('manifest')
    diagnosis_evidence = evidence.get('diagnosis')
    verify_evidence = evidence.get('verify_log')
    if not all(isinstance(value, dict) for value in (
        manifest_evidence,
        diagnosis_evidence,
        verify_evidence,
    )):
        return False
    recorded_hashes = _manifest_artifact_hashes(manifest)
    return all((
        receipt.get('status') == 'PASS',
        manifest.get('status') == 'succeeded',
        diagnosis.get('status') == 'success',
        verify_log_status == 'PASS',
        lifecycle.get('stage') == 'complete',
        lifecycle.get('runner_exit_code') == 0,
        output.get('finalized') is True,
        verification.get('manifest_status') == 'succeeded',
        verification.get('diagnosis_status') == 'success',
        verification.get('autoware_status') == 'PASS',
        verification.get('manifest_sha256') == manifest_sha256,
        manifest_evidence.get('filename') == 'run_manifest.json',
        manifest_evidence.get('sha256') == manifest_sha256,
        diagnosis_evidence.get('filename')
        == 'autoware_map_diagnosis.json',
        diagnosis_evidence.get('available') is True,
        diagnosis_evidence.get('sha256') == diagnosis_sha256,
        verify_evidence.get('filename') == 'verify_autoware_map.log',
        verify_evidence.get('available') is True,
        verify_evidence.get('sha256') == verify_log_sha256,
        recorded_hashes.get('autoware_map_diagnosis.json')
        == diagnosis_sha256,
        recorded_hashes.get('verify_autoware_map.log')
        == verify_log_sha256,
        run.get('run_id') == manifest.get('run_id'),
        run.get('product_version') == software.get('product_version'),
        run.get('profile_id') == profile.get('id'),
    ))


def _artifact_state(trial_dir: Path) -> dict[str, Any]:
    output_root = trial_dir / 'output'
    final_dir = output_root / 'mid360_demo'
    partial_dir = output_root / 'mid360_demo.partial'
    run_dir = final_dir if final_dir.is_dir() else partial_dir
    if not run_dir.is_dir():
        run_dir = None

    manifest_path = run_dir / 'run_manifest.json' if run_dir else None
    diagnosis_path = (
        run_dir / 'autoware_map_diagnosis.json' if run_dir else None
    )
    receipt_path = (
        run_dir / 'first_map_validation_receipt.json' if run_dir else None
    )
    verify_log_path = run_dir / 'verify_autoware_map.log' if run_dir else None
    manifest = _load_json(manifest_path) if manifest_path else None
    diagnosis = _load_json(diagnosis_path) if diagnosis_path else None
    receipt = _load_json(receipt_path) if receipt_path else None
    manifest_sha256 = _sha256(manifest_path) if manifest_path else None
    diagnosis_sha256 = _sha256(diagnosis_path) if diagnosis_path else None
    verify_log_sha256 = _sha256(verify_log_path) if verify_log_path else None
    verify_status = _verify_log_status(verify_log_path)
    receipt_semantic_pass = _receipt_semantically_passes(
        receipt,
        manifest,
        diagnosis,
        manifest_sha256,
        diagnosis_sha256,
        verify_log_sha256,
        verify_status,
    )

    manifest_status = 'missing'
    if manifest_path and manifest_path.is_file():
        manifest_status = (
            'succeeded'
            if manifest and manifest.get('status') == 'succeeded'
            else 'failed'
        )
    diagnosis_status = 'missing'
    if diagnosis_path and diagnosis_path.is_file():
        diagnosis_status = (
            'success'
            if diagnosis and diagnosis.get('status') == 'success'
            else 'failure'
        )
    receipt_status = 'NOT_CREATED'
    verifier_status = (
        verify_status if verify_status in {'PASS', 'FAIL'} else 'NOT_RUN'
    )
    if receipt_path and receipt_path.is_file():
        receipt_status = 'PASS' if receipt_semantic_pass else 'FAIL'
    receipt_run = receipt.get('run') if receipt else None
    if not isinstance(receipt_run, dict):
        receipt_run = {}

    return {
        'run_dir': run_dir,
        'manifest_path': manifest_path,
        'receipt_path': receipt_path,
        'manifest_status': manifest_status,
        'diagnosis_status': diagnosis_status,
        'verifier_status': verifier_status,
        'receipt_status': receipt_status,
        'receipt_semantic_pass': receipt_semantic_pass,
        'product_version': receipt_run.get('product_version'),
        'profile_id': receipt_run.get('profile_id'),
        'manifest_sha256': manifest_sha256,
        'receipt_sha256': _sha256(receipt_path) if receipt_path else None,
    }


def _failure_details(
    runner_exit_code: int,
    artifact: dict[str, Any],
    archive_bytes: int,
    timed_out: bool,
    receipt_observed: bool,
) -> tuple[str, list[str]]:
    if timed_out:
        if receipt_observed:
            return 'receipt', ['runner-timeout-after-receipt']
        if archive_bytes == 0 and artifact['manifest_status'] == 'missing':
            return 'download', ['download-timeout']
        return 'mapping', ['mapping-timeout']
    if archive_bytes == 0 and artifact['manifest_status'] == 'missing':
        return 'download', ['docker-or-dataset-download-failed']
    if artifact['manifest_status'] == 'missing':
        return 'mapping', ['run-manifest-missing']
    if artifact['manifest_status'] != 'succeeded':
        return 'mapping', ['mapping-failed']
    if artifact['diagnosis_status'] != 'success':
        return 'verification', ['diagnosis-failed']
    if artifact['verifier_status'] != 'PASS':
        return 'verification', ['autoware-verifier-failed']
    if artifact['receipt_status'] == 'NOT_CREATED':
        return 'receipt', ['receipt-missing']
    if artifact['receipt_status'] != 'PASS':
        finding = (
            'receipt-semantic-invalid'
            if not artifact['receipt_semantic_pass']
            else 'receipt-failed'
        )
        return 'receipt', [finding]
    if runner_exit_code != 0:
        return 'receipt', ['runner-exit-after-receipt']
    return 'receipt', ['unknown-route-failure']


def _allocated_output_bytes(
    host_name: str,
    run_dir: Path | None,
    trial_dir: Path,
) -> int:
    if run_dir is None:
        return 0
    relative = run_dir.relative_to(trial_dir)
    container_path = '/trial/' + relative.as_posix()
    result = _docker_exec(
        host_name,
        'du',
        '-sx',
        '--block-size=1',
        container_path,
    )
    first = result.stdout.strip().split(maxsplit=1)[0]
    try:
        return int(first)
    except ValueError as exc:
        raise ProbeError('cannot parse allocated output bytes') from exc


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True) + '\n'
    with path.open('x', encoding='utf-8') as stream:
        stream.write(payload)


def _retain_validation_receipt(
    artifact: dict[str, Any],
    output: Path | None,
) -> None:
    """Copy the exact shareable PASS receipt outside private evidence."""
    if output is None or not artifact['receipt_semantic_pass']:
        return
    source = artifact['receipt_path']
    if (
        source is None
        or source.is_symlink()
        or not source.is_file()
    ):
        raise ProbeError('validated first-map receipt is not a regular file')
    try:
        payload = source.read_bytes()
        if hashlib.sha256(payload).hexdigest() != artifact['receipt_sha256']:
            raise ProbeError(
                'validated first-map receipt changed before retention'
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('xb') as stream:
            stream.write(payload)
    except FileExistsError as exc:
        raise ProbeError(
            f'refusing to overwrite validation receipt: {output}'
        ) from exc
    except OSError as exc:
        raise ProbeError(
            f'cannot retain validation receipt {output}: {exc}'
        ) from exc


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _read_rx(host_name: str, interface: str) -> int:
    path = f'/sys/class/net/{interface}/statistics/rx_bytes'
    result = _docker_exec(host_name, 'cat', path)
    try:
        return int(result.stdout.strip())
    except ValueError as exc:
        raise ProbeError('cannot parse isolated interface RX counter') from exc


def _stream_output(
    stream: TextIO,
    log_stream: TextIO,
) -> None:
    for line in iter(stream.readline, ''):
        log_stream.write(line)
        log_stream.flush()
        print(line, end='', flush=True)
    stream.close()


def _wait_for_daemon(host_name: str, timeout_sec: float = 90.0) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        result = _docker_exec(
            host_name,
            'docker',
            'info',
            check=False,
            timeout_sec=5.0,
        )
        if result.returncode == 0:
            return
        time.sleep(1.0)
    logs = _docker(
        'logs', host_name, check=False, timeout_sec=10.0
    )
    raise ProbeError(
        'nested Docker daemon did not become ready: ' + logs.stderr
    )


def _validate_outer_daemon() -> None:
    overrides = [
        name for name in ('DOCKER_HOST', 'DOCKER_CONTEXT')
        if os.environ.get(name)
    ]
    if overrides:
        raise ProbeError(
            'refusing Docker environment override: ' + ', '.join(overrides)
        )
    context = _docker(
        'context', 'show', timeout_sec=10.0
    ).stdout.strip()
    if context != 'default':
        raise ProbeError(
            f'refusing non-default Docker context: {context or "missing"}'
        )
    endpoint = _docker(
        'context',
        'inspect',
        'default',
        '--format',
        '{{.Endpoints.docker.Host}}',
        timeout_sec=10.0,
    ).stdout.strip()
    if endpoint != OUTER_DOCKER_ENDPOINT:
        raise ProbeError(
            'refusing non-local Docker endpoint for privileged probe'
        )


def _validate_outer_container(
    host_name: str,
    container_id: str,
    observer_image: str,
    observer_image_id: str,
    docker_dir: Path,
    trial_dir: Path,
) -> None:
    inspect_result = _docker(
        'inspect', host_name, timeout_sec=10.0
    )
    try:
        values = json.loads(inspect_result.stdout)
    except json.JSONDecodeError as exc:
        raise ProbeError('cannot parse outer container inspection') from exc
    if not isinstance(values, list) or len(values) != 1:
        raise ProbeError('outer container inspection is not singular')
    value = values[0]
    if not isinstance(value, dict):
        raise ProbeError('outer container inspection is not an object')
    if value.get('Id') != container_id:
        raise ProbeError('outer container ID does not match its cidfile')
    if value.get('Name') != f'/{host_name}':
        raise ProbeError('outer container name changed unexpectedly')
    if value.get('Image') != observer_image_id:
        raise ProbeError('outer container image ID changed unexpectedly')

    host_config = value.get('HostConfig')
    config = value.get('Config')
    mounts = value.get('Mounts')
    network_settings = value.get('NetworkSettings')
    if not all(isinstance(item, dict) for item in (
        host_config,
        config,
        network_settings,
    )) or not isinstance(mounts, list):
        raise ProbeError('outer container host configuration is missing')
    assert isinstance(config, dict)
    assert isinstance(network_settings, dict)
    if config.get('Image') != observer_image:
        raise ProbeError('outer container image reference changed')
    if host_config.get('Privileged') is not True:
        raise ProbeError('outer container is not in the acknowledged mode')
    if host_config.get('NetworkMode') != 'bridge':
        raise ProbeError('outer container network is not isolated bridge mode')
    networks = network_settings.get('Networks')
    if not isinstance(networks, dict) or set(networks) != {'bridge'}:
        raise ProbeError(
            'outer container has an unexpected network attachment'
        )

    expected = {
        '/var/lib/docker': docker_dir.resolve(),
        '/trial': trial_dir.resolve(),
    }
    if len(mounts) != len(expected):
        raise ProbeError('outer container has an unexpected mount count')
    for mount in mounts:
        if not isinstance(mount, dict):
            raise ProbeError('outer container mount is malformed')
        destination = mount.get('Destination')
        source = mount.get('Source')
        if destination not in expected:
            raise ProbeError('outer container has an unexpected mount')
        if mount.get('Type') != 'bind' or mount.get('RW') is not True:
            raise ProbeError('outer container bind mode is unexpected')
        if mount.get('Propagation') != 'rprivate':
            raise ProbeError('outer container bind is not rprivate')
        if Path(str(source)).resolve() != expected[destination]:
            raise ProbeError('outer container bind source changed')


def _remove_outer_host(container_id: str) -> None:
    result = _docker(
        'rm',
        '-f',
        container_id,
        check=False,
        timeout_sec=DOCKER_CONTROL_TIMEOUT_SEC,
    )
    if result.returncode != 0:
        raise ProbeError('failed to remove the named outer container')
    inspection = _docker(
        'inspect',
        container_id,
        check=False,
        timeout_sec=10.0,
    )
    if inspection.returncode == 0:
        raise ProbeError('outer container still exists after removal')
    if 'no such object' not in inspection.stderr.lower():
        raise ProbeError('outer container removal could not be confirmed')


def _inner_container_ids(
    host_name: str,
    *,
    include_stopped: bool,
) -> list[str]:
    arguments = ['docker', 'ps', '--no-trunc', '-q']
    if include_stopped:
        arguments.append('-a')
    result = _docker_exec(
        host_name,
        *arguments,
        check=False,
    )
    if result.returncode != 0:
        raise ProbeError('cannot inspect nested product containers')
    container_ids = result.stdout.split()
    if not all(
        re.fullmatch(r'[0-9a-f]{64}', value)
        for value in container_ids
    ):
        raise ProbeError('nested daemon returned a malformed container ID')
    return container_ids


def _cleanup_timed_out_inner_containers(host_name: str) -> None:
    for inner_id in _inner_container_ids(
        host_name,
        include_stopped=False,
    ):
        stopped = _docker_exec(
            host_name,
            'docker',
            'stop',
            '--time',
            '10',
            inner_id,
            check=False,
            timeout_sec=20.0,
        )
        if (
            stopped.returncode != 0
            and inner_id in _inner_container_ids(
                host_name,
                include_stopped=True,
            )
        ):
            raise ProbeError('timed-out inner container did not stop')

    for inner_id in _inner_container_ids(
        host_name,
        include_stopped=True,
    ):
        removed = _docker_exec(
            host_name,
            'docker',
            'rm',
            '-f',
            inner_id,
            check=False,
            timeout_sec=20.0,
        )
        if (
            removed.returncode != 0
            and inner_id in _inner_container_ids(
                host_name,
                include_stopped=True,
            )
        ):
            raise ProbeError('timed-out inner container was not removed')

    if _inner_container_ids(host_name, include_stopped=True):
        raise ProbeError('timed-out inner container cleanup is incomplete')


def _validate_nested_host(host_name: str, expected_os: str) -> str:
    info_result = _docker_exec(
        host_name,
        'docker',
        'info',
        '--format',
        '{{json .}}',
    )
    try:
        info = json.loads(info_result.stdout)
    except json.JSONDecodeError as exc:
        raise ProbeError('cannot parse nested docker info') from exc
    expected = {
        'Architecture': 'x86_64',
        'Driver': 'overlay2',
        'DockerRootDir': '/var/lib/docker',
        'Images': 0,
        'Containers': 0,
    }
    mismatches = [
        f'{key}={info.get(key)!r} (expected {value!r})'
        for key, value in expected.items()
        if info.get(key) != value
    ]
    if mismatches:
        raise ProbeError(
            'unclean or unsupported nested daemon: '
            + '; '.join(mismatches)
        )

    os_release = _docker_exec(host_name, 'cat', '/etc/os-release').stdout
    expected_version = f'VERSION_ID="{expected_os}"'
    if 'ID=ubuntu\n' not in os_release or expected_version not in os_release:
        raise ProbeError(f'nested host is not Ubuntu {expected_os}')

    route = _docker_exec(host_name, 'ip', '-o', 'route', 'show', 'default')
    fields = route.stdout.strip().split()
    try:
        interface = fields[fields.index('dev') + 1]
    except (ValueError, IndexError) as exc:
        raise ProbeError('cannot resolve isolated default interface') from exc
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', interface):
        raise ProbeError('unsafe network interface name')
    _read_rx(host_name, interface)
    return interface


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Run an isolated Docker first-map machine probe and write a '
            'privacy-bounded onboarding-trial v1 record.'
        ),
    )
    parser.add_argument('--trial-id', required=True)
    parser.add_argument(
        '--ros-distro', required=True, choices=sorted(OS_VERSION)
    )
    image_identity = parser.add_mutually_exclusive_group(required=True)
    image_identity.add_argument('--image-tag')
    image_identity.add_argument('--candidate-image-ref')
    parser.add_argument('--image-digest', required=True)
    parser.add_argument('--product-version', default='0.9.0')
    parser.add_argument('--candidate-image-set-sha256')
    parser.add_argument('--candidate-evidence-bundle-sha256')
    parser.add_argument('--candidate-source-pr', type=int)
    parser.add_argument('--candidate-source-commit')
    parser.add_argument('--candidate-workflow-run-url')
    parser.add_argument('--record', required=True, type=Path)
    parser.add_argument(
        '--validation-receipt-output',
        type=Path,
        help=(
            'Retain the exact privacy-bounded PASS first-map receipt here; '
            'the destination is never overwritten.'
        ),
    )
    parser.add_argument('--temp-parent', default='/tmp', type=Path)
    parser.add_argument(
        '--disk-scope',
        type=Path,
        help=(
            'Dedicated host filesystem to sample for peak disk usage. '
            'Omit on a shared host; the record then remains non-comparable.'
        ),
    )
    parser.add_argument('--timeout-sec', default=7200.0, type=float)
    parser.add_argument(
        '--prompt-human-measurements',
        action='store_true',
        help=(
            'Prompt for both observed active operator time and human-'
            'submitted command count.'
        ),
    )
    parser.add_argument(
        '--record-human-measurements-unknown',
        action='store_true',
        help=(
            'Record both human measurements as unknown; the trial remains '
            'valid but non-comparable.'
        ),
    )
    parser.add_argument(
        '--prompt-active-operator-time',
        action='store_true',
        help='Prompt for the observed active operator time only.',
    )
    parser.add_argument(
        '--record-active-time-unknown',
        action='store_true',
        help='Leave active operator time unknown explicitly.',
    )
    parser.add_argument(
        '--prompt-command-count',
        action='store_true',
        help='Prompt for the human-submitted command count only.',
    )
    parser.add_argument(
        '--record-command-count-unknown',
        action='store_true',
        help='Leave command count unknown explicitly.',
    )
    parser.add_argument(
        '--allow-privileged-container-host',
        action='store_true',
        help=(
            'Acknowledge that the disposable nested Docker host uses '
            '--privileged. Prefer running it inside a dedicated VM.'
        ),
    )
    parser.add_argument(
        '--build-observer-image-if-missing',
        action='store_true',
        help=(
            'build the reviewed local observer image before timing when its '
            'exact tag is absent; never replace an existing tag'
        ),
    )
    parser.add_argument(
        '--observer-image',
        help=(
            'reviewed local observer image tag; intended for the candidate '
            'session wrapper'
        ),
    )
    parser.add_argument(
        '--observer-recipe-sha256',
        help='exact SHA-256 of the reviewed observer Dockerfile',
    )
    parser.add_argument(
        '--acknowledge-dedicated-filesystem',
        action='store_true',
        help=(
            'Acknowledge that --disk-scope is a disposable dedicated VM '
            'filesystem with no unrelated activity.'
        ),
    )
    args = parser.parse_args(argv)
    if not args.allow_privileged_container_host:
        parser.error(
            '--allow-privileged-container-host is required; review the '
            'isolation warning first'
        )
    if args.observer_image is not None and OBSERVER_IMAGE_RE.fullmatch(
        args.observer_image
    ) is None:
        parser.error('--observer-image is not a bounded local trial-host tag')
    if args.observer_recipe_sha256 is not None and SHA256_RE.fullmatch(
        args.observer_recipe_sha256
    ) is None:
        parser.error(
            '--observer-recipe-sha256 must be 64 lower-case hex digits'
        )
    if (
        args.observer_image is not None
        and args.observer_recipe_sha256 is None
    ):
        parser.error('--observer-image requires --observer-recipe-sha256')
    if args.disk_scope is None and args.acknowledge_dedicated_filesystem:
        parser.error(
            '--acknowledge-dedicated-filesystem requires --disk-scope'
        )
    if (
        args.disk_scope is not None
        and not args.acknowledge_dedicated_filesystem
    ):
        parser.error(
            '--disk-scope requires --acknowledge-dedicated-filesystem; '
            'shared-host disk measurements are not comparable'
        )
    if not TRIAL_ID_RE.fullmatch(args.trial_id):
        parser.error('--trial-id must be a privacy-bounded lower-case slug')
    if not DIGEST_RE.fullmatch(args.image_digest):
        parser.error(
            '--image-digest must be sha256 followed by 64 lower-case hex '
            'digits'
        )
    if not VERSION_RE.fullmatch(args.product_version):
        parser.error('--product-version must be a semantic version')
    candidate_fields = {
        '--candidate-image-set-sha256': args.candidate_image_set_sha256,
        '--candidate-evidence-bundle-sha256': (
            args.candidate_evidence_bundle_sha256
        ),
        '--candidate-source-pr': args.candidate_source_pr,
        '--candidate-source-commit': args.candidate_source_commit,
        '--candidate-workflow-run-url': args.candidate_workflow_run_url,
    }
    if args.image_tag is not None:
        match = IMAGE_TAG_RE.fullmatch(args.image_tag)
        if not match or match.group('distro') != args.ros_distro:
            parser.error(
                '--image-tag must be the matching immutable release image '
                'tag'
            )
        if match.group('version') != args.product_version:
            parser.error(
                '--product-version must match the release image tag'
            )
        supplied_candidate_fields = [
            name for name, value in candidate_fields.items()
            if value is not None
        ]
        if supplied_candidate_fields:
            parser.error(
                'release image mode cannot include candidate evidence: '
                + ', '.join(supplied_candidate_fields)
            )
        args.resolved_image_ref = f'{args.image_tag}@{args.image_digest}'
        args.candidate_image_evidence = None
    else:
        match = CANDIDATE_IMAGE_REF_RE.fullmatch(args.candidate_image_ref)
        if match is None:
            parser.error(
                '--candidate-image-ref must be the exact tag-free GHCR '
                'repository@sha256 reference'
            )
        if match.group('digest') != args.image_digest:
            parser.error(
                '--candidate-image-ref digest must match --image-digest'
            )
        missing_candidate_fields = [
            name for name, value in candidate_fields.items()
            if value is None
        ]
        if missing_candidate_fields:
            parser.error(
                'candidate image mode requires: '
                + ', '.join(missing_candidate_fields)
            )
        if SHA256_RE.fullmatch(args.candidate_image_set_sha256) is None:
            parser.error(
                '--candidate-image-set-sha256 must be 64 lower-case hex '
                'digits'
            )
        if SHA256_RE.fullmatch(
            args.candidate_evidence_bundle_sha256
        ) is None:
            parser.error(
                '--candidate-evidence-bundle-sha256 must be 64 lower-case '
                'hex digits'
            )
        if args.candidate_source_pr < 1:
            parser.error('--candidate-source-pr must be a positive integer')
        if COMMIT_RE.fullmatch(args.candidate_source_commit) is None:
            parser.error(
                '--candidate-source-commit must be a lower-case '
                '40-character commit SHA'
            )
        if WORKFLOW_RUN_URL_RE.fullmatch(
            args.candidate_workflow_run_url
        ) is None:
            parser.error(
                '--candidate-workflow-run-url must identify an exact '
                'candidate workflow run'
            )
        args.resolved_image_ref = args.candidate_image_ref
        args.candidate_image_evidence = {
            'sha256': args.candidate_image_set_sha256,
            'bundle_sha256': args.candidate_evidence_bundle_sha256,
            'source_pr': args.candidate_source_pr,
            'source_commit': args.candidate_source_commit,
            'product_version': args.product_version,
            'workflow_run_url': args.candidate_workflow_run_url,
            'immutable_ref': args.candidate_image_ref,
        }
    if args.prompt_human_measurements:
        args.prompt_active_operator_time = True
        args.prompt_command_count = True
    if args.record_human_measurements_unknown:
        args.record_active_time_unknown = True
        args.record_command_count_unknown = True
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error('--timeout-sec must be finite and greater than zero')
    if args.prompt_active_operator_time and args.record_active_time_unknown:
        parser.error('active-time modes are mutually exclusive')
    if args.prompt_command_count and args.record_command_count_unknown:
        parser.error('command-count modes are mutually exclusive')
    if not args.temp_parent.is_dir():
        parser.error('--temp-parent must be an existing directory')
    if args.validation_receipt_output is not None:
        if args.validation_receipt_output == args.record:
            parser.error(
                '--validation-receipt-output must differ from --record'
            )
        if os.path.lexists(args.validation_receipt_output):
            parser.error(
                '--validation-receipt-output already exists'
            )
    if args.disk_scope is not None:
        if args.disk_scope.is_symlink() or not args.disk_scope.is_dir():
            parser.error('--disk-scope must be an existing real directory')
        args.disk_scope = args.disk_scope.resolve()
    return args


def run_probe(args: argparse.Namespace) -> tuple[dict[str, Any], Path, Path]:
    """Execute one isolated machine probe and return its bounded record."""
    if args.record.exists():
        raise ProbeError(
            f'refusing to overwrite existing record: {args.record}'
        )
    _validate_outer_daemon()
    os_version = OS_VERSION[args.ros_distro]
    observer_image, observer_image_id = _ensure_observer_image(
        args, os_version
    )

    name_hash = hashlib.sha256(args.trial_id.encode()).hexdigest()[:12]
    host_name = f'lidarslam-g0-{args.ros_distro}-{name_hash}'
    existing = _docker(
        'ps', '-aq', '--filter', f'name=^/{host_name}$', check=False
    ).stdout.strip()
    if existing:
        raise ProbeError(f'refusing to reuse existing container {host_name}')

    trial_root = Path(tempfile.mkdtemp(
        prefix=f'lidarslam-g0-{args.trial_id}-',
        dir=args.temp_parent,
    )).resolve()
    observer_root = Path(tempfile.mkdtemp(
        prefix=f'lidarslam-g0-observer-{args.trial_id}-',
        dir=args.temp_parent,
    )).resolve()
    docker_dir = trial_root / 'docker'
    trial_dir = trial_root / 'trial'
    for directory in (
        docker_dir,
        trial_dir / 'data',
        trial_dir / 'output',
    ):
        directory.mkdir(parents=True, exist_ok=False)

    disk_sampler: DiskSampler | None = None
    disk_baseline: int | None = None
    sampler_stopped = False
    if args.disk_scope is not None:
        scope_device = args.disk_scope.stat().st_dev
        if any(
            path.stat().st_dev != scope_device
            for path in (trial_root, docker_dir, trial_dir)
        ):
            raise ProbeError(
                '--disk-scope must contain the trial and nested Docker data '
                'on one filesystem'
            )
        disk_sampler = DiskSampler(args.disk_scope)

    print(f'private trial root: {trial_root}', file=sys.stderr)
    print(f'private observer root: {observer_root}', file=sys.stderr)
    cidfile = observer_root / 'outer-container.cid'
    container_id: str | None = None
    record: dict[str, Any] | None = None
    try:
        mount_docker = (
            f'type=bind,src={docker_dir},dst=/var/lib/docker,'
            'bind-propagation=rprivate'
        )
        mount_trial = (
            f'type=bind,src={trial_dir},dst=/trial,'
            'bind-propagation=rprivate'
        )
        run_result = _docker(
            'run', '-d',
            '--name', host_name,
            '--cidfile', str(cidfile),
            '--privileged',
            '--network', 'bridge',
            '--mount', mount_docker,
            '--mount', mount_trial,
            observer_image,
        )
        run_container_id = run_result.stdout.strip()
        if not re.fullmatch(r'[0-9a-f]{64}', run_container_id):
            raise ProbeError('docker run did not return a container ID')
        container_id = run_container_id
        try:
            cidfile_id = cidfile.read_text(encoding='utf-8').strip()
        except OSError as exc:
            raise ProbeError(
                'outer container cidfile was not created'
            ) from exc
        if not re.fullmatch(r'[0-9a-f]{64}', cidfile_id):
            raise ProbeError('outer container cidfile is malformed')
        if cidfile_id != container_id:
            raise ProbeError('docker run and cidfile container IDs differ')
        _validate_outer_container(
            host_name,
            container_id,
            observer_image,
            observer_image_id,
            docker_dir,
            trial_dir,
        )
        _wait_for_daemon(host_name)
        interface = _validate_nested_host(host_name, os_version)

        image_ref = args.resolved_image_ref
        if disk_sampler is not None:
            disk_baseline = disk_sampler.start()
        rx_start = _read_rx(host_name, interface)
        start_time = time.monotonic()
        stop_time: float | None = None
        rx_end: int | None = None
        timed_out = False
        receipt_observed = False
        post_receipt_deadline: float | None = None

        command = [
            'docker', '--host', OUTER_DOCKER_ENDPOINT,
            'exec', host_name,
            'docker', 'run', '--rm',
            '-e', 'DEMO_DATA_DIR=/trial/data',
            '-e', 'DEMO_OUTPUT_DIR=/trial/output/mid360_demo',
            '-e', 'LIDARSLAM_HOST_UID=0',
            '-e', 'LIDARSLAM_HOST_GID=0',
            '--mount',
            'type=bind,src=/trial,dst=/trial,bind-propagation=rprivate',
            image_ref,
        ]
        log_path = observer_root / 'product-route.log'
        with log_path.open('w', encoding='utf-8') as log_stream:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            if process.stdout is None:
                raise ProbeError('product route stdout pipe was not created')
            output_thread = threading.Thread(
                target=_stream_output,
                args=(process.stdout, log_stream),
                daemon=True,
            )
            output_thread.start()
            receipt_candidates = (
                trial_dir / 'output' / 'mid360_demo'
                / 'first_map_validation_receipt.json',
                trial_dir / 'output' / 'mid360_demo.partial'
                / 'first_map_validation_receipt.json',
            )
            deadline = start_time + args.timeout_sec
            while process.poll() is None:
                now = time.monotonic()
                receipt_created = any(
                    path.is_file() for path in receipt_candidates
                )
                if not receipt_observed and receipt_created:
                    receipt_observed = True
                    stop_time = now
                    rx_end = _read_rx(host_name, interface)
                    post_receipt_deadline = (
                        now + POST_RECEIPT_GRACE_SEC
                    )
                timeout_deadline = deadline
                if post_receipt_deadline is not None:
                    timeout_deadline = min(
                        timeout_deadline,
                        post_receipt_deadline,
                    )
                if now >= timeout_deadline:
                    timed_out = True
                    if stop_time is None:
                        stop_time = now
                        rx_end = _read_rx(host_name, interface)
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired as exc:
                            raise ProbeError(
                                'docker exec client did not terminate'
                            ) from exc
                    break
                time.sleep(0.1)
            runner_exit_code = process.wait()
            output_thread.join(timeout=10)
            if output_thread.is_alive():
                raise ProbeError('product output reader did not stop')
            if not receipt_observed and any(
                path.is_file() for path in receipt_candidates
            ):
                receipt_observed = True
                stop_time = time.monotonic()
                rx_end = _read_rx(host_name, interface)

        if disk_sampler is not None and not sampler_stopped:
            disk_sampler.stop()
            sampler_stopped = True

        if timed_out:
            _cleanup_timed_out_inner_containers(host_name)
        elif _inner_container_ids(host_name, include_stopped=True):
            raise ProbeError(
                'product route returned with a nested container remaining'
            )

        if stop_time is None:
            stop_time = time.monotonic()
            rx_end = _read_rx(host_name, interface)
        if disk_sampler is not None and not sampler_stopped:
            disk_sampler.stop()
            sampler_stopped = True
        if rx_end is None or rx_end < rx_start:
            raise ProbeError('isolated interface RX counter moved backwards')
        peak_disk = (
            disk_sampler.peak_delta(disk_baseline)
            if disk_sampler is not None and disk_baseline is not None
            else None
        )

        archive = trial_dir / ARCHIVE_RELATIVE
        archive_part = archive.with_suffix(archive.suffix + '.part')
        if archive.is_file():
            archive_bytes = archive.stat().st_size
        elif archive_part.is_file():
            archive_bytes = archive_part.stat().st_size
        else:
            archive_bytes = 0

        artifact = _artifact_state(trial_dir)
        output_bytes = _allocated_output_bytes(
            host_name,
            artifact['run_dir'],
            trial_dir,
        )
        pass_outcome = (
            not timed_out
            and runner_exit_code == 0
            and artifact['manifest_status'] == 'succeeded'
            and artifact['diagnosis_status'] == 'success'
            and artifact['verifier_status'] == 'PASS'
            and artifact['receipt_status'] == 'PASS'
            and artifact['receipt_semantic_pass']
            and artifact['product_version'] == args.product_version
            and artifact['profile_id'] == 'rko_lio_graph_mid360_preset'
            and artifact['manifest_sha256'] is not None
            and artifact['receipt_sha256'] is not None
        )
        if pass_outcome:
            failure_stage = 'none'
            finding_codes: list[str] = []
        else:
            failure_stage, finding_codes = _failure_details(
                runner_exit_code,
                artifact,
                archive_bytes,
                timed_out,
                receipt_observed,
            )

        wall_time = round(stop_time - start_time, 3)
        active_time = _prompt_active_time(
            wall_time,
            getattr(args, 'record_active_time_unknown', False),
        )
        command_count = _prompt_command_count(
            getattr(args, 'record_command_count_unknown', False),
        )

        record = {
            'schema_version': 1,
            'schema_uri': SCHEMA_URI,
            'trial_id': args.trial_id,
            'captured_at': _utc_now(),
            'documentation_path': 'docker-first-map',
            'operator_class': 'maintainer',
            'environment': {
                'clean_start': True,
                'ros_distro': args.ros_distro,
                'architecture': 'x86_64',
                'os_family': f'ubuntu-{os_version}',
                'product_version': args.product_version,
                'revision': {
                    'kind': 'image-digest',
                    'value': args.image_digest,
                },
            },
            'input': {
                'dataset_class': 'fixed-public',
                'dataset_id': 'mid360-public-zenodo-14841855',
                'download_bytes': archive_bytes,
            },
            'measurements': {
                'workflow_download_bytes': rx_end - rx_start,
                'wall_time_sec': wall_time,
                'active_operator_time_sec': active_time,
                'command_count': command_count,
                'peak_disk_bytes': peak_disk,
                'output_bytes': output_bytes,
            },
            'outcome': {
                'status': 'PASS' if pass_outcome else 'FAIL',
                'runner_exit_code': runner_exit_code,
                'manifest_status': artifact['manifest_status'],
                'diagnosis_status': artifact['diagnosis_status'],
                'verifier_status': artifact['verifier_status'],
                'receipt_status': artifact['receipt_status'],
                'undocumented_manual_steps': 0,
                'failure_stage': failure_stage,
                'finding_codes': finding_codes,
            },
            'evidence': {
                'manifest_sha256': artifact['manifest_sha256'],
                'receipt_sha256': artifact['receipt_sha256'],
            },
            'privacy': {
                'contains_private_paths': False,
                'contains_exact_command': False,
                'contains_operator_identity': False,
                'review_before_sharing': True,
            },
        }
        if args.candidate_image_evidence is not None:
            record['evidence']['candidate_image_set'] = (
                args.candidate_image_evidence
            )
    finally:
        try:
            if disk_sampler is not None and not sampler_stopped:
                disk_sampler.stop()
                sampler_stopped = True
        finally:
            cleanup_id = container_id
            if cleanup_id is None and cidfile.is_file():
                try:
                    candidate = cidfile.read_text(encoding='utf-8').strip()
                except OSError as exc:
                    raise ProbeError(
                        'cannot read outer container cidfile for cleanup'
                    ) from exc
                if not re.fullmatch(r'[0-9a-f]{64}', candidate):
                    raise ProbeError(
                        'outer container cleanup cidfile is malformed'
                    )
                cleanup_id = candidate
            if cleanup_id is not None:
                _remove_outer_host(cleanup_id)

    if record is None:
        raise ProbeError('probe ended without a bounded record')
    _retain_validation_receipt(
        artifact,
        getattr(args, 'validation_receipt_output', None),
    )
    _write_json(args.record, record)
    _write_json(observer_root / 'bounded-record.json', record)
    return record, trial_root, observer_root


def main(argv: list[str] | None = None) -> int:
    """Run the probe; PASS exits 0, route FAIL 1, and harness error 2."""
    args = _parse_args(argv)
    try:
        record, trial_root, observer_root = run_probe(args)
    except (OSError, ProbeError, subprocess.SubprocessError) as exc:
        print(f'probe error: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(record, indent=2, sort_keys=True))
    print(f'private trial root retained: {trial_root}', file=sys.stderr)
    print(f'private observer root retained: {observer_root}', file=sys.stderr)
    return 0 if record['outcome']['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
