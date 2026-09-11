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

"""Measure one clean source quickstart on an acknowledged disposable host.

The bounded JSON record contains only aggregate measurements and artifact
hashes. Exact commands, paths, host output, and disk samples stay in the
private observer directory printed by the script.
"""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Callable, TextIO
import urllib.error
import urllib.parse
import urllib.request

try:
    from github_api_auth import github_api_authorization
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import github_api_authorization

# Planning and public-route inspection must not mutate the observer checkout
# merely by importing adjacent helper modules.
sys.dont_write_bytecode = True


SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-trial-v1.schema.json'
)
REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = REPO_ROOT / 'docs' / 'schemas' / 'onboarding-trial-v1.schema.json'
REPO_URL = 'https://github.com/rsasaki0109/lidar_slam_ros2.git'
GITHUB_API = 'https://api.github.com/repos/rsasaki0109/lidar_slam_ros2'
SOURCE_COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
TRIAL_ID_RE = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')
VERSION_RE = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$'
)
INTERFACE_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
OS_VERSION = {'humble': '22.04', 'jazzy': '24.04'}
EXPECTED_SOURCE_PACKAGES = (
    'graph_based_slam',
    'lidarslam',
    'lidarslam_msgs',
    'ndt_omp_ros2',
    'rko_lio',
    'scanmatcher',
)
ARCHIVE_BYTES = 517088133
ARCHIVE_RELATIVE = Path(
    'datasets/mid360_public/driving_slam_mid360/archives/'
    'rosbag2_2024_04_16-14_17_01.zip'
)
POST_RECEIPT_GRACE_SEC = 60.0
SAMPLE_INTERVAL_SEC = 0.25


class ProbeError(RuntimeError):
    """The observer harness cannot produce trustworthy evidence."""


class RouteUnavailable(RuntimeError):
    """The public source route is honestly unavailable before execution."""

    def __init__(self, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code


def evaluate_trial(
    record: dict[str, Any],
    schema: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load the shared trial evaluator without writing adjacent bytecode."""
    from check_onboarding_trial import evaluate_trial as evaluate

    return evaluate(
        record,
        schema,
        require_evidence_binding=False,
    )


def _artifact_state(root: Path) -> dict[str, Any]:
    """Load the shared artifact inspector only for a measured execution."""
    from run_docker_onboarding_probe import _artifact_state as inspect

    return inspect(root)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProbeError(f'cannot read JSON contract {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise ProbeError(f'JSON contract root is not an object: {path}')
    return value


def _write_json_exclusive(path: Path, value: dict[str, Any]) -> None:
    payload = json.dumps(value, indent=2, sort_keys=True) + '\n'
    try:
        with path.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    except FileExistsError as exc:
        raise ProbeError(
            f'refusing to overwrite existing record: {path}'
        ) from exc


def _retain_validation_receipt(
    artifact: dict[str, Any],
    output: Path | None,
) -> None:
    """Copy the exact shareable PASS receipt outside private evidence."""
    if output is None or not artifact['receipt_semantic_pass']:
        return
    source = artifact['receipt_path']
    if source is None or source.is_symlink() or not source.is_file():
        raise ProbeError('validated first-map receipt is not a regular file')
    try:
        payload = source.read_bytes()
        if hashlib.sha256(payload).hexdigest() != artifact['receipt_sha256']:
            raise ProbeError(
                'validated first-map receipt changed before retention'
            )
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


def _contains(parent: Path, child: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def _meaningful_entries(path: Path) -> list[Path]:
    entries = list(path.iterdir())
    return [item for item in entries if item.name != 'lost+found']


def _validate_paths(args: argparse.Namespace) -> None:
    for label, path in (
        ('trial root', args.trial_root),
        ('observer parent', args.observer_parent),
        ('disk scope', args.disk_scope),
    ):
        if path.is_symlink() or not path.is_dir():
            raise ProbeError(
                f'{label} must be an existing real directory: {path}'
            )
    args.trial_root = args.trial_root.resolve()
    args.observer_parent = args.observer_parent.resolve()
    args.disk_scope = args.disk_scope.resolve()
    requested_record = args.record.expanduser().absolute()
    if os.path.lexists(requested_record):
        raise ProbeError(
            f'refusing to overwrite existing record: {requested_record}'
        )
    if requested_record.parent.is_symlink() or not requested_record.parent.is_dir():
        raise ProbeError(
            'record parent must be one existing non-symlink directory: '
            f'{requested_record.parent}'
        )
    args.record = requested_record.parent.resolve(strict=True) / requested_record.name
    validation_receipt = getattr(
        args,
        'validation_receipt_output',
        None,
    )
    if validation_receipt is not None:
        validation_receipt = validation_receipt.absolute()
        args.validation_receipt_output = validation_receipt
    if args.trial_root in {Path('/'), Path.home().resolve()}:
        raise ProbeError('trial root must not be / or the user home directory')
    entries = _meaningful_entries(args.trial_root)
    if entries:
        raise ProbeError('trial root is not empty: ' + ', '.join(
            sorted(item.name for item in entries)[:10]
        ))
    if (
        _contains(args.trial_root, args.observer_parent)
        or _contains(args.observer_parent, args.trial_root)
    ):
        raise ProbeError('trial root and observer parent must not overlap')
    if _contains(args.trial_root, args.record):
        raise ProbeError(
            'bounded record must be outside the private trial root'
        )
    if not args.record.parent.is_dir():
        raise ProbeError(f'record parent does not exist: {args.record.parent}')
    if os.path.lexists(args.record):
        raise ProbeError(
            f'refusing to overwrite existing record: {args.record}'
        )
    if validation_receipt is not None:
        if validation_receipt == args.record:
            raise ProbeError(
                'validation receipt output must differ from the trial record'
            )
        if _contains(args.trial_root, validation_receipt):
            raise ProbeError(
                'validation receipt output must be outside the private '
                'trial root'
            )
        if not validation_receipt.parent.is_dir():
            raise ProbeError(
                'validation receipt parent does not exist: '
                f'{validation_receipt.parent}'
            )
        if os.path.lexists(validation_receipt):
            raise ProbeError(
                'refusing to overwrite validation receipt: '
                f'{validation_receipt}'
            )
    scope_device = args.disk_scope.stat().st_dev
    measured_paths = (args.trial_root, Path('/usr'), Path('/var'))
    if any(path.stat().st_dev != scope_device for path in measured_paths):
        raise ProbeError(
            'disk scope must cover the trial root, /usr, and /var on one '
            'disposable-host filesystem'
        )


def _os_version(path: Path = Path('/etc/os-release')) -> str:
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError as exc:
        raise ProbeError(f'cannot read host OS identity: {exc}') from exc
    for line in lines:
        key, separator, value = line.partition('=')
        if key == 'VERSION_ID' and separator:
            return value.strip().strip('\"\'')
    raise ProbeError('host OS identity has no VERSION_ID')


def _validate_host(ros_distro: str) -> None:
    expected_os = OS_VERSION[ros_distro]
    actual_os = _os_version()
    if actual_os != expected_os:
        raise ProbeError(
            f'{ros_distro} trial requires Ubuntu {expected_os}; found {actual_os}'
        )
    architecture = platform.machine()
    if architecture != 'x86_64':
        raise ProbeError(f'source matrix requires x86_64; found {architecture}')
    setup = Path('/opt/ros') / ros_distro / 'setup.bash'
    if not setup.is_file():
        raise ProbeError(f'ROS setup is missing: {setup}')
    for command in ('bash', 'df', 'du', 'git', 'ip'):
        if not shutil_which(command):
            raise ProbeError(f'required observer command is unavailable: {command}')


def shutil_which(command: str) -> str | None:
    """Keep command lookup mockable without importing a broad utility module."""
    paths = os.environ.get('PATH', '').split(os.pathsep)
    for directory in paths:
        candidate = Path(directory) / command
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def _network_interface(requested: str | None) -> str:
    if requested:
        interface = requested
    else:
        result = subprocess.run(
            ['ip', '-o', 'route', 'show', 'default'],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            raise ProbeError('cannot inspect the default network interface')
        fields = result.stdout.split()
        try:
            interface = fields[fields.index('dev') + 1]
        except (ValueError, IndexError) as exc:
            raise ProbeError('cannot resolve the default network interface') from exc
    if not INTERFACE_RE.fullmatch(interface):
        raise ProbeError('network interface name is unsafe')
    _read_rx(interface)
    return interface


def _read_rx(interface: str) -> int:
    path = Path('/sys/class/net') / interface / 'statistics' / 'rx_bytes'
    try:
        value = int(path.read_text(encoding='utf-8').strip())
    except (OSError, ValueError) as exc:
        raise ProbeError(f'cannot read RX counter for {interface}') from exc
    if value < 0:
        raise ProbeError('network RX counter is negative')
    return value


def _disk_used_bytes(scope: Path) -> int:
    result = subprocess.run(
        ['df', '--output=used', '-B1', str(scope)],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        raise ProbeError(f'cannot measure disk scope: {scope}')
    lines = result.stdout.splitlines()
    try:
        value = int(lines[1].strip())
    except (IndexError, ValueError) as exc:
        raise ProbeError('cannot parse allocated disk usage') from exc
    if value < 0:
        raise ProbeError('allocated disk usage is negative')
    return value


class _DiskSampler:
    def __init__(
        self,
        scope: Path,
        reader: Callable[[Path], int] = _disk_used_bytes,
    ) -> None:
        self.scope = scope
        self.reader = reader
        self.samples: list[tuple[float, int]] = []
        self.error: BaseException | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> int:
        baseline = self.reader(self.scope)
        self.samples.append((time.time(), baseline))
        self._thread = threading.Thread(target=self._sample, daemon=True)
        self._thread.start()
        return baseline

    def _sample(self) -> None:
        try:
            while not self._stop.wait(SAMPLE_INTERVAL_SEC):
                self.samples.append((time.time(), self.reader(self.scope)))
        except BaseException as exc:  # propagated on the controlling thread
            self.error = exc
            self._stop.set()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise ProbeError('disk sampler did not stop')
        if self.error is not None:
            raise ProbeError(f'disk sampler failed: {self.error}')

    def peak_delta(self, baseline: int) -> int:
        if not self.samples:
            raise ProbeError('disk sampler produced no observations')
        delta = max(value for _, value in self.samples) - baseline
        if delta < 0:
            raise ProbeError('peak disk allocation moved below baseline')
        return delta


def _request_json(url: str) -> tuple[int, dict[str, Any] | None]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-source-onboarding-probe/1',
    }
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(
        url,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            body = response.read().decode('utf-8')
            payload = json.loads(body)
            if not isinstance(payload, dict):
                raise ProbeError('GitHub response root is not an object')
            return response.status, payload
    except urllib.error.HTTPError as exc:
        # GitHub returns 404 for some unresolved refs and 422 for an unknown
        # but syntactically valid 40-character commit passed to the commits
        # endpoint. Callers retain endpoint context and decide whether either
        # status proves an unavailable route or an observer/API problem.
        if exc.code in {404, 422}:
            return exc.code, None
        raise ProbeError(f'GitHub inspection returned HTTP {exc.code}') from exc
    except (
        urllib.error.URLError,
        TimeoutError,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as exc:
        raise ProbeError(f'GitHub inspection failed: {exc}') from exc


def _content_at(commit: str, relative: str) -> str:
    encoded = urllib.parse.quote(relative, safe='/')
    status, payload = _request_json(
        f'{GITHUB_API}/contents/{encoded}?ref={commit}'
    )
    if status == 404:
        raise RouteUnavailable(
            'source-route-contract-missing',
            f'public commit lacks {relative}',
        )
    if status != 200 or payload is None:
        raise ProbeError(
            f'GitHub content inspection returned HTTP {status} for {relative}')
    if payload.get('type') != 'file' or not isinstance(payload.get('content'), str):
        raise ProbeError(f'GitHub content response is malformed for {relative}')
    try:
        encoded_content = ''.join(payload['content'].split())
        return base64.b64decode(encoded_content, validate=True).decode('utf-8')
    except (ValueError, UnicodeDecodeError) as exc:
        raise ProbeError(f'cannot decode public content for {relative}') from exc


def _require_markers(
    content: str,
    markers: tuple[str, ...],
    label: str,
) -> None:
    missing = [marker for marker in markers if marker not in content]
    if missing:
        raise RouteUnavailable(
            'source-route-contract-missing',
            f'public {label} lacks required route marker {missing[0]!r}',
        )


def _shell_array(content: str, name: str) -> tuple[str, ...]:
    match = re.search(
        rf'(?ms)^{re.escape(name)}=\(\s*(.*?)^\)',
        content,
    )
    if match is None:
        raise RouteUnavailable(
            'source-route-contract-missing',
            f'public source quickstart lacks {name}',
        )
    try:
        values = tuple(shlex.split(match.group(1), comments=True))
    except ValueError as exc:
        raise ProbeError(f'cannot parse public {name}: {exc}') from exc
    return values


def _preflight_public_source(
    commit: str,
    product_version: str,
) -> dict[str, Any]:
    status, payload = _request_json(f'{GITHUB_API}/commits/{commit}')
    if status in {404, 422}:
        raise RouteUnavailable(
            'source-candidate-not-published',
            'exact source commit is not publicly resolvable',
        )
    if status != 200 or payload is None:
        raise ProbeError(
            f'GitHub commit inspection returned unexpected HTTP {status}')
    if payload.get('sha') != commit:
        raise ProbeError('GitHub commit identity does not match the request')
    helper = _content_at(commit, 'scripts/source_quickstart.sh')
    packages = _shell_array(helper, 'EXPECTED_SOURCE_PACKAGES')
    if packages != EXPECTED_SOURCE_PACKAGES:
        raise RouteUnavailable(
            'source-route-contract-missing',
            'public source quickstart package inventory differs from the '
            'maintained six-package contract',
        )
    _require_markers(
        helper,
        (
            '--packages-select "${EXPECTED_SOURCE_PACKAGES[@]}"',
            '[source-package-inventory-mismatch]',
            'install_source_dependencies.sh',
            '--repo-only',
            '-DBUILD_TESTING=OFF',
            'set +u\nsource "${ROS_SETUP}"\nset -u',
            'set +u\nsource "${INSTALL_SETUP}"\nset -u',
            'lidarslam-map demo',
        ),
        'source quickstart',
    )
    dependency_helper = _content_at(
        commit,
        'scripts/install_source_dependencies.sh',
    )
    _require_markers(
        dependency_helper,
        (
            '--repo-only',
            'rosdep install',
            '--from-paths "${DEPENDENCY_ROOT}"',
            '--ignore-src',
        ),
        'source dependency helper',
    )
    getting_started = _content_at(commit, 'docs/getting-started.md')
    _require_markers(
        getting_started,
        (
            'bash scripts/source_quickstart.sh',
            '6 ROS packages',
            'BUILD_TESTING=OFF',
            'without changing your shell',
        ),
        'Getting Started',
    )
    version = _content_at(commit, 'VERSION').strip()
    if version != product_version:
        raise RouteUnavailable(
            'source-version-mismatch',
            f'public VERSION is {version!r}, expected {product_version!r}',
        )
    return {
        'commit': commit,
        'product_version': version,
        'source_packages': list(packages),
        'contract_files': [
            'scripts/source_quickstart.sh',
            'scripts/install_source_dependencies.sh',
            'docs/getting-started.md',
            'VERSION',
        ],
    }


def _public_preflight_report(
    commit: str,
    product_version: str,
) -> dict[str, Any]:
    try:
        details = _preflight_public_source(commit, product_version)
    except RouteUnavailable as unavailable:
        return {
            'schema_version': 1,
            'status': 'NOT_READY',
            'repository': REPO_URL,
            'source_commit': commit,
            'product_version': product_version,
            'network_requested': True,
            'writes_performed': False,
            'finding_codes': [unavailable.code],
            'detail': str(unavailable),
        }
    return {
        'schema_version': 1,
        'status': 'READY',
        'repository': REPO_URL,
        'source_commit': commit,
        'product_version': product_version,
        'network_requested': True,
        'writes_performed': False,
        'finding_codes': [],
        'details': details,
    }


def _base_record(args: argparse.Namespace) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'trial_id': args.trial_id,
        'captured_at': _utc_now(),
        'documentation_path': 'source-quickstart',
        'operator_class': 'maintainer',
        'environment': {
            'clean_start': True,
            'ros_distro': args.ros_distro,
            'architecture': 'x86_64',
            'os_family': f'ubuntu-{OS_VERSION[args.ros_distro]}',
            'product_version': args.product_version,
            'revision': {'kind': 'git-commit', 'value': args.source_commit},
        },
        'input': {
            'dataset_class': 'fixed-public',
            'dataset_id': 'mid360-public-zenodo-14841855',
            'download_bytes': None,
        },
        'measurements': {
            'workflow_download_bytes': None,
            'wall_time_sec': None,
            'active_operator_time_sec': None,
            'command_count': None,
            'peak_disk_bytes': None,
            'output_bytes': None,
        },
        'outcome': {
            'status': 'FAIL',
            'runner_exit_code': None,
            'manifest_status': 'missing',
            'diagnosis_status': 'missing',
            'verifier_status': 'NOT_RUN',
            'receipt_status': 'NOT_CREATED',
            'undocumented_manual_steps': 0,
            'failure_stage': 'preflight',
            'finding_codes': [],
        },
        'evidence': {'manifest_sha256': None, 'receipt_sha256': None},
        'privacy': {
            'contains_private_paths': False,
            'contains_exact_command': False,
            'contains_operator_identity': False,
            'review_before_sharing': True,
        },
    }


def _preflight_failure_record(
    args: argparse.Namespace,
    unavailable: RouteUnavailable,
) -> dict[str, Any]:
    record = _base_record(args)
    record['outcome']['finding_codes'] = [unavailable.code]
    return record


def _route_script(args: argparse.Namespace, observer_root: Path) -> Path:
    trial = shlex.quote(str(args.trial_root))
    repository = shlex.quote(str(args.trial_root / 'src' / 'lidar_slam_ros2'))
    commit = shlex.quote(args.source_commit)
    distro = shlex.quote(args.ros_distro)
    script = observer_root / 'source-route.sh'
    content = f"""#!/usr/bin/env bash
set -Eeuo pipefail
mkdir -p {trial}/src
git -c init.defaultBranch=detached init {repository}
git -C {repository} remote add origin {shlex.quote(REPO_URL)}
git -C {repository} fetch --depth=1 origin {commit}
git -C {repository} checkout --detach FETCH_HEAD
test "$(git -C {repository} rev-parse HEAD)" = {commit}
git -C {repository} submodule update --init --recursive
test "$(tr -d '\\n' < {repository}/VERSION)" = {shlex.quote(args.product_version)}
cd {repository}
exec bash scripts/source_quickstart.sh \\
  --workspace {trial} --ros-distro {distro} --viewer none
"""
    try:
        with script.open('x', encoding='utf-8') as stream:
            stream.write(content)
        script.chmod(0o700)
    except OSError as exc:
        raise ProbeError(f'cannot create private source route: {exc}') from exc
    return script


def _stream_output(stream: TextIO, log: TextIO) -> None:
    for line in iter(stream.readline, ''):
        log.write(line)
        log.flush()
        print(line, end='', flush=True)
    stream.close()


def _terminate_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired as exc:
            raise ProbeError('source route process group did not terminate') from exc


def _allocated_output_bytes(run_dir: Path | None) -> int:
    if run_dir is None:
        return 0
    result = subprocess.run(
        ['du', '-sx', '--block-size=1', str(run_dir)],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        raise ProbeError('cannot measure allocated map output')
    try:
        return int(result.stdout.split(maxsplit=1)[0])
    except (IndexError, ValueError) as exc:
        raise ProbeError('cannot parse allocated map output') from exc


def _log_has_marker(
    path: Path | None,
    marker: str,
    max_bytes: int = 1024 * 1024,
) -> bool:
    if path is None or not path.is_file():
        return False
    try:
        with path.open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - max_bytes))
            tail = stream.read(max_bytes)
    except OSError as exc:
        raise ProbeError(f'cannot inspect private source route log: {exc}') from exc
    return marker.encode() in tail


def _failure_details(
    args: argparse.Namespace,
    artifact: dict[str, Any],
    archive_bytes: int,
    timed_out: bool,
    route_log: Path | None = None,
) -> tuple[str, list[str]]:
    install_setup = args.trial_root / 'install' / 'setup.bash'
    checkout = args.trial_root / 'src' / 'lidar_slam_ros2'
    if timed_out:
        if not install_setup.is_file():
            return 'install', ['source-install-timeout']
        if archive_bytes < ARCHIVE_BYTES and artifact['manifest_status'] == 'missing':
            return 'download', ['dataset-download-timeout']
        return 'mapping', ['mapping-timeout']
    if not checkout.is_dir():
        return 'install', ['source-clone-failed']
    if not install_setup.is_file():
        if _log_has_marker(
            route_log,
            '[source-package-inventory-mismatch]',
        ):
            return 'install', ['source-package-inventory-mismatch']
        return 'install', ['source-install-failed']
    if archive_bytes == 0 and artifact['manifest_status'] == 'missing':
        return 'download', ['dataset-download-failed']
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
        return 'receipt', ['receipt-semantic-invalid']
    return 'receipt', ['source-route-exit-after-valid-receipt']


def _prompt_active_time(wall_time: float, unknown: bool) -> float | None:
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
    """Capture the human-submitted command count without inferring it."""
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


def _write_disk_samples(path: Path, samples: list[tuple[float, int]]) -> None:
    try:
        with path.open('x', encoding='utf-8') as stream:
            for captured_at, value in samples:
                stream.write(f'{captured_at:.9f}\t{value}\n')
    except OSError as exc:
        raise ProbeError(f'cannot retain private disk samples: {exc}') from exc


def _validate_record(record: dict[str, Any]) -> None:
    import jsonschema

    schema = _load_object(SCHEMA_PATH)
    try:
        jsonschema.Draft7Validator(schema).validate(record)
        evaluate_trial(record, schema)
    except (jsonschema.ValidationError, ValueError) as exc:
        raise ProbeError(f'generated onboarding record is invalid: {exc}') from exc


def _dry_run_plan(args: argparse.Namespace, interface: str) -> dict[str, Any]:
    return {
        'mode': 'dry-run',
        'network_or_writes_performed': False,
        'trial_id': args.trial_id,
        'ros_distro': args.ros_distro,
        'source_commit': args.source_commit,
        'product_version': args.product_version,
        'repository': REPO_URL,
        'trial_root': str(args.trial_root),
        'observer_parent': str(args.observer_parent),
        'disk_scope': str(args.disk_scope),
        'network_interface': interface,
        'record': str(args.record),
        'stages': [
            'verify_public_source_identity',
            'start_disk_and_network_measurement',
            'clone_exact_source',
            'run_source_quickstart_headless',
            'validate_first_map_receipt',
            'capture_observed_active_time',
            'capture_observed_command_count',
            'write_privacy_bounded_trial_record',
        ],
    }


def run_probe(args: argparse.Namespace) -> tuple[dict[str, Any], Path | None]:
    """Run one route or return a bounded preflight failure."""
    _validate_paths(args)
    _validate_host(args.ros_distro)
    interface = _network_interface(args.network_interface)
    if args.dry_run:
        print(json.dumps(_dry_run_plan(args, interface), indent=2, sort_keys=True))
        return {}, None
    if not args.acknowledge_disposable_host:
        raise ProbeError(
            '--acknowledge-disposable-host is required because APT and the '
            'source build modify the host'
        )
    if not args.acknowledge_isolated_network:
        raise ProbeError(
            '--acknowledge-isolated-network is required before treating RX '
            'as a workflow measurement'
        )
    if not (args.prompt_active_operator_time or args.record_active_time_unknown):
        raise ProbeError(
            'choose --prompt-human-measurements, '
            '--prompt-active-operator-time, or '
            '--record-active-time-unknown'
        )
    if not (args.prompt_command_count or args.record_command_count_unknown):
        raise ProbeError(
            'choose --prompt-human-measurements, '
            '--prompt-command-count, or '
            '--record-command-count-unknown'
        )

    try:
        _preflight_public_source(args.source_commit, args.product_version)
    except RouteUnavailable as unavailable:
        record = _preflight_failure_record(args, unavailable)
        _validate_record(record)
        _write_json_exclusive(args.record, record)
        print(f'route unavailable: [{unavailable.code}] {unavailable}', file=sys.stderr)
        return record, None

    observer_root = Path(tempfile.mkdtemp(
        prefix=f'lidarslam-source-observer-{args.trial_id}-',
        dir=args.observer_parent,
    )).resolve()
    print(f'private observer root: {observer_root}', file=sys.stderr)
    route_script = _route_script(args, observer_root)
    sampler = _DiskSampler(args.disk_scope)
    baseline = sampler.start()
    rx_start = _read_rx(interface)
    start_time = time.monotonic()
    stop_time: float | None = None
    rx_end: int | None = None
    timed_out = False
    sampler_stopped = False

    log_path = observer_root / 'source-route.log'
    process: subprocess.Popen[str] | None = None
    try:
        with log_path.open('x', encoding='utf-8') as log:
            process = subprocess.Popen(
                ['bash', str(route_script)],
                cwd=args.trial_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            if process.stdout is None:
                raise ProbeError('source route output pipe was not created')
            output_thread = threading.Thread(
                target=_stream_output,
                args=(process.stdout, log),
                daemon=True,
            )
            output_thread.start()
            receipt_candidates = (
                args.trial_root / 'output' / 'mid360_demo'
                / 'first_map_validation_receipt.json',
                args.trial_root / 'output' / 'mid360_demo.partial'
                / 'first_map_validation_receipt.json',
            )
            deadline = start_time + args.timeout_sec
            post_receipt_deadline: float | None = None
            while process.poll() is None:
                now = time.monotonic()
                if stop_time is None and any(
                    path.is_file() for path in receipt_candidates
                ):
                    stop_time = now
                    rx_end = _read_rx(interface)
                    sampler.stop()
                    sampler_stopped = True
                    post_receipt_deadline = now + POST_RECEIPT_GRACE_SEC
                effective_deadline = deadline
                if post_receipt_deadline is not None:
                    effective_deadline = min(
                        effective_deadline, post_receipt_deadline
                    )
                if now >= effective_deadline:
                    timed_out = True
                    if stop_time is None:
                        stop_time = now
                        rx_end = _read_rx(interface)
                        sampler.stop()
                        sampler_stopped = True
                    _terminate_process_group(process)
                    break
                time.sleep(0.1)
            runner_exit_code = process.wait()
            output_thread.join(timeout=10)
            if output_thread.is_alive():
                raise ProbeError('source output reader did not stop')
    finally:
        if process is not None and process.poll() is None:
            _terminate_process_group(process)
        if not sampler_stopped:
            sampler.stop()
            sampler_stopped = True

    if stop_time is None:
        stop_time = time.monotonic()
        rx_end = _read_rx(interface)
    if rx_end is None or rx_end < rx_start:
        raise ProbeError('network RX counter moved backwards')
    wall_time = round(stop_time - start_time, 3)
    peak_disk = sampler.peak_delta(baseline)
    _write_disk_samples(observer_root / 'disk.tsv', sampler.samples)

    archive = args.trial_root / ARCHIVE_RELATIVE
    archive_part = archive.with_suffix(archive.suffix + '.part')
    if archive.is_file():
        archive_bytes = archive.stat().st_size
    elif archive_part.is_file():
        archive_bytes = archive_part.stat().st_size
    else:
        archive_bytes = 0
    artifact = _artifact_state(args.trial_root)
    output_bytes = _allocated_output_bytes(artifact['run_dir'])
    pass_outcome = all((
        not timed_out,
        runner_exit_code == 0,
        archive_bytes == ARCHIVE_BYTES,
        artifact['manifest_status'] == 'succeeded',
        artifact['diagnosis_status'] == 'success',
        artifact['verifier_status'] == 'PASS',
        artifact['receipt_status'] == 'PASS',
        artifact['receipt_semantic_pass'],
        artifact['product_version'] == args.product_version,
        artifact['profile_id'] == 'rko_lio_graph_mid360_preset',
        artifact['manifest_sha256'] is not None,
        artifact['receipt_sha256'] is not None,
    ))
    if pass_outcome:
        failure_stage = 'none'
        finding_codes: list[str] = []
    else:
        failure_stage, finding_codes = _failure_details(
            args,
            artifact,
            archive_bytes,
            timed_out,
            log_path,
        )
    active_time = _prompt_active_time(
        wall_time,
        args.record_active_time_unknown,
    )
    command_count = _prompt_command_count(
        args.record_command_count_unknown,
    )

    record = _base_record(args)
    record['input']['download_bytes'] = archive_bytes
    record['measurements'] = {
        'workflow_download_bytes': rx_end - rx_start,
        'wall_time_sec': wall_time,
        'active_operator_time_sec': active_time,
        'command_count': command_count,
        'peak_disk_bytes': peak_disk,
        'output_bytes': output_bytes,
    }
    record['outcome'] = {
        'status': 'PASS' if pass_outcome else 'FAIL',
        'runner_exit_code': runner_exit_code,
        'manifest_status': artifact['manifest_status'],
        'diagnosis_status': artifact['diagnosis_status'],
        'verifier_status': artifact['verifier_status'],
        'receipt_status': artifact['receipt_status'],
        'undocumented_manual_steps': 0,
        'failure_stage': failure_stage,
        'finding_codes': finding_codes,
    }
    record['evidence'] = {
        'manifest_sha256': artifact['manifest_sha256'],
        'receipt_sha256': artifact['receipt_sha256'],
    }
    _validate_record(record)
    _retain_validation_receipt(
        artifact,
        getattr(args, 'validation_receipt_output', None),
    )
    _write_json_exclusive(args.record, record)
    _write_json_exclusive(observer_root / 'bounded-record.json', record)
    return record, observer_root


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Run the exact public source quickstart on a disposable host and '
            'write an onboarding-trial v1 record.'
        ),
    )
    parser.add_argument('--trial-id')
    parser.add_argument('--ros-distro', choices=sorted(OS_VERSION))
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--product-version', required=True)
    parser.add_argument('--trial-root', type=Path)
    parser.add_argument('--observer-parent', type=Path)
    parser.add_argument('--disk-scope', type=Path)
    parser.add_argument('--network-interface')
    parser.add_argument('--record', type=Path)
    parser.add_argument(
        '--validation-receipt-output',
        type=Path,
        help=(
            'Retain the exact privacy-bounded PASS first-map receipt here; '
            'the destination is never overwritten.'
        ),
    )
    parser.add_argument('--timeout-sec', type=float, default=7200.0)
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
    parser.add_argument('--acknowledge-disposable-host', action='store_true')
    parser.add_argument('--acknowledge-isolated-network', action='store_true')
    parser.add_argument(
        '--public-preflight',
        action='store_true',
        help=(
            'Query only the public immutable source route and print JSON; '
            'do not require a trial host or write files.'
        ),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print a read-only plan; do not query GitHub or write files.',
    )
    args = parser.parse_args(argv)
    if not SOURCE_COMMIT_RE.fullmatch(args.source_commit):
        parser.error('--source-commit must be exactly 40 lower-case hex digits')
    if not VERSION_RE.fullmatch(args.product_version):
        parser.error('--product-version must be a semantic version')
    if args.prompt_human_measurements:
        args.prompt_active_operator_time = True
        args.prompt_command_count = True
    if args.record_human_measurements_unknown:
        args.record_active_time_unknown = True
        args.record_command_count_unknown = True
    if args.public_preflight:
        execution_options = (
            args.trial_id,
            args.ros_distro,
            args.trial_root,
            args.observer_parent,
            args.disk_scope,
            args.network_interface,
            args.record,
            args.validation_receipt_output,
        )
        if any(value is not None for value in execution_options):
            parser.error(
                '--public-preflight accepts only source identity options'
            )
        if (
            args.dry_run
            or args.prompt_active_operator_time
            or args.record_active_time_unknown
            or args.prompt_command_count
            or args.record_command_count_unknown
            or args.acknowledge_disposable_host
            or args.acknowledge_isolated_network
        ):
            parser.error(
                '--public-preflight cannot be combined with execution modes'
            )
        return args
    required_execution_options = {
        '--trial-id': args.trial_id,
        '--ros-distro': args.ros_distro,
        '--trial-root': args.trial_root,
        '--observer-parent': args.observer_parent,
        '--disk-scope': args.disk_scope,
        '--record': args.record,
    }
    missing = [
        option
        for option, value in required_execution_options.items()
        if value is None
    ]
    if missing:
        parser.error('execution mode requires ' + ', '.join(missing))
    if not TRIAL_ID_RE.fullmatch(args.trial_id):
        parser.error('--trial-id must be a privacy-bounded lower-case slug')
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error('--timeout-sec must be finite and greater than zero')
    if args.prompt_active_operator_time and args.record_active_time_unknown:
        parser.error('active-time modes are mutually exclusive')
    if args.prompt_command_count and args.record_command_count_unknown:
        parser.error('command-count modes are mutually exclusive')
    return args


def main(argv: list[str] | None = None) -> int:
    """PASS exits 0, route FAIL exits 1, and harness error exits 2."""
    args = _parse_args(argv)
    if args.public_preflight:
        try:
            report = _public_preflight_report(
                args.source_commit,
                args.product_version,
            )
        except (OSError, ProbeError, subprocess.SubprocessError) as exc:
            print(f'probe error: {exc}', file=sys.stderr)
            return 2
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report['status'] == 'READY' else 1
    try:
        record, observer_root = run_probe(args)
    except (OSError, ProbeError, subprocess.SubprocessError) as exc:
        print(f'probe error: {exc}', file=sys.stderr)
        return 2
    if args.dry_run:
        return 0
    print(json.dumps(record, indent=2, sort_keys=True))
    if observer_root is not None:
        print(
            f'private observer root retained: {observer_root}',
            file=sys.stderr,
        )
    return 0 if record['outcome']['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
