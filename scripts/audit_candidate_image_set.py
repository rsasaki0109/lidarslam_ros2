#!/usr/bin/env python3
"""Audit one retained immutable candidate-image evidence bundle.

The four-file bundle is checked locally without network access. Remote mode
then downloads the exact four artifacts from the recorded workflow run into a
temporary directory and compares their bytes before checking GHCR manifests
and GitHub attestations. No remote mutation is authorized or performed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any, Callable, Sequence

from create_candidate_image_record import build_candidate_image_record

from product_schema import load_json_object, validate_contract

from validate_candidate_image_request import (
    ALLOWED_SKIPPED_CHECKS,
    REQUIRED_SUCCESS_CHECKS,
)

from verify_candidate_image_set import verify_candidate_image_set


CANDIDATE_SET_SCHEMA = 'candidate-image-set-v1.schema.json'
REQUEST_SCHEMA = 'candidate-image-request-v1.schema.json'
AUDIT_SCHEMA = 'candidate-image-set-audit-v2.schema.json'
AUDIT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-image-set-audit-v2.schema.json'
)
EXPECTED_REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
EXPECTED_WORKFLOW_PATH = '.github/workflows/candidate-image.yml'
MAX_EVIDENCE_FILE_BYTES = 1024 * 1024
EVIDENCE_FILES = (
    (
        'candidate-image-request',
        'candidate-image-request.json',
        'request',
    ),
    (
        'candidate-image-record-humble',
        'candidate-image-humble.json',
        'humble',
    ),
    (
        'candidate-image-record-jazzy',
        'candidate-image-jazzy.json',
        'jazzy',
    ),
    (
        'candidate-image-set',
        'candidate-image-set.json',
        'set',
    ),
)
EXPECTED_ARTIFACTS = tuple(item[0] for item in EVIDENCE_FILES)
EXPECTED_FILENAMES = tuple(item[1] for item in EVIDENCE_FILES)
RUN_URL_RE = re.compile(
    r'^https://github\.com/rsasaki0109/lidar_slam_ros2/actions/'
    r'runs/(?P<run_id>[1-9][0-9]*)$'
)
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateSetAuditError(ValueError):
    """Candidate evidence cannot be interpreted without guessing."""


def sha256_file(path: Path) -> str:
    """Hash one retained evidence file without loading arbitrary-size input."""
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise CandidateSetAuditError(
            f'candidate evidence cannot be hashed: {exc}'
        ) from exc
    return digest.hexdigest()


def candidate_bundle_sha256(file_hashes: dict[str, str]) -> str:
    """Hash the canonical filename-to-file-hash manifest for one bundle."""
    if set(file_hashes) != set(EXPECTED_FILENAMES):
        raise CandidateSetAuditError(
            'candidate evidence bundle requires all four canonical hashes'
        )
    lines = []
    for filename in EXPECTED_FILENAMES:
        value = file_hashes[filename]
        if SHA256_RE.fullmatch(value) is None:
            raise CandidateSetAuditError(
                f'candidate evidence hash is malformed: {filename}'
            )
        lines.append(f'{filename}\t{value}\n')
    return hashlib.sha256(''.join(lines).encode('utf-8')).hexdigest()


def _validate_bounded_file(path: Path, label: str) -> None:
    try:
        if path.is_symlink() or not path.is_file():
            raise CandidateSetAuditError(
                f'{label} must be one regular, non-symlink file'
            )
        size = path.stat().st_size
    except OSError as exc:
        raise CandidateSetAuditError(f'{label} cannot be inspected: {exc}') \
            from exc
    if size > MAX_EVIDENCE_FILE_BYTES:
        raise CandidateSetAuditError(
            f'{label} exceeds {MAX_EVIDENCE_FILE_BYTES} bytes'
        )


def _validate_request_semantics(request: dict[str, Any]) -> None:
    validate_contract(request, REQUEST_SCHEMA)
    required = sorted(REQUIRED_SUCCESS_CHECKS)
    if request['required_success_checks'] != required:
        raise CandidateSetAuditError(
            'candidate request required-check contract does not match'
        )
    observed_success = set(request['observed_successful_checks'])
    if not REQUIRED_SUCCESS_CHECKS.issubset(observed_success):
        raise CandidateSetAuditError(
            'candidate request omits a required successful check'
        )
    if not set(request['observed_skipped_checks']).issubset(
        ALLOWED_SKIPPED_CHECKS
    ):
        raise CandidateSetAuditError(
            'candidate request contains an unexpected skipped check'
        )


def validate_candidate_set(
    candidate_set: dict[str, Any],
) -> list[dict[str, str]]:
    """Require one self-consistent tag-free image for each distro."""
    validate_contract(candidate_set, CANDIDATE_SET_SCHEMA)
    if candidate_set['repository'] != EXPECTED_REPOSITORY:
        raise CandidateSetAuditError('candidate repository is unexpected')
    images = candidate_set['images']
    by_distro = {image['ros_distro']: image for image in images}
    if len(by_distro) != 2 or set(by_distro) != {'humble', 'jazzy'}:
        raise CandidateSetAuditError(
            'candidate set requires Humble and Jazzy exactly once'
        )
    ordered = [by_distro['humble'], by_distro['jazzy']]
    if len({image['digest'] for image in ordered}) != 2:
        raise CandidateSetAuditError(
            'Humble and Jazzy candidate digests must differ'
        )
    repository_ref = f"ghcr.io/{candidate_set['repository']}"
    for image in ordered:
        expected_ref = f"{repository_ref}@{image['digest']}"
        if image['immutable_ref'] != expected_ref:
            raise CandidateSetAuditError(
                f"{image['ros_distro']} immutable ref does not match digest"
            )
    run_match = RUN_URL_RE.fullmatch(candidate_set['workflow_run_url'])
    if run_match is None:
        raise CandidateSetAuditError('candidate workflow run URL is malformed')
    return ordered


def load_candidate_evidence_bundle(directory: Path) -> dict[str, Any]:
    """Load and bind the exact request, records, and set in one directory."""
    try:
        if directory.is_symlink() or not directory.is_dir():
            raise CandidateSetAuditError(
                'candidate evidence path must be one real directory'
            )
        entries = sorted(item.name for item in directory.iterdir())
    except OSError as exc:
        raise CandidateSetAuditError(
            f'candidate evidence directory cannot be inspected: {exc}'
        ) from exc
    if entries != sorted(EXPECTED_FILENAMES):
        raise CandidateSetAuditError(
            'candidate evidence directory must contain exactly: '
            + ', '.join(EXPECTED_FILENAMES)
        )

    paths = {filename: directory / filename for filename in EXPECTED_FILENAMES}
    for filename, path in paths.items():
        _validate_bounded_file(path, filename)
    request = load_json_object(
        paths['candidate-image-request.json'],
        'candidate image request',
    )
    records = {
        distro: load_json_object(
            paths[f'candidate-image-{distro}.json'],
            f'candidate image {distro} record',
        )
        for distro in ('humble', 'jazzy')
    }
    retained_set = load_json_object(
        paths['candidate-image-set.json'],
        'candidate image set',
    )
    _validate_request_semantics(request)

    regenerated_records: list[dict[str, Any]] = []
    for distro in ('humble', 'jazzy'):
        record = records[distro]
        regenerated = build_candidate_image_record(
            request,
            ros_distro=distro,
            platform=record.get('platform'),
            digest=record.get('digest'),
            cli_version=record.get('cli_version'),
            workflow_run_url=record.get('workflow_run_url'),
            evidence_retention_days=record.get('evidence_retention_days'),
        )
        if regenerated != record:
            raise CandidateSetAuditError(
                f'candidate {distro} record does not derive from request'
            )
        regenerated_records.append(regenerated)
    regenerated_set = verify_candidate_image_set(regenerated_records)
    if regenerated_set != retained_set:
        raise CandidateSetAuditError(
            'candidate set does not derive exactly from both image records'
        )
    validate_candidate_set(retained_set)

    file_hashes = {
        filename: sha256_file(path) for filename, path in paths.items()
    }
    return {
        'directory': directory,
        'paths': paths,
        'request': request,
        'records': records,
        'candidate_set': retained_set,
        'file_hashes': file_hashes,
        'bundle_sha256': candidate_bundle_sha256(file_hashes),
    }


def _run(
    command: list[str],
    *,
    runner: Runner,
) -> subprocess.CompletedProcess[str]:
    try:
        return runner(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return subprocess.CompletedProcess(command, 125, '', str(exc))


def _json_object(
    result: subprocess.CompletedProcess[str],
) -> dict[str, Any] | None:
    if result.returncode != 0:
        return None
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def _retained_files(
    file_hashes: dict[str, str] | None,
) -> list[dict[str, Any]]:
    if file_hashes is None:
        return [{
            'artifact_name': 'candidate-image-set',
            'filename': 'candidate-image-set.json',
            'sha256': '',
            'remote_content_status': 'NOT_CHECKED',
        }]
    return [
        {
            'artifact_name': artifact_name,
            'filename': filename,
            'sha256': file_hashes[filename],
            'remote_content_status': 'NOT_CHECKED',
        }
        for artifact_name, filename, _key in EVIDENCE_FILES
    ]


def _base_report(
    candidate_set: dict[str, Any],
    *,
    candidate_set_sha256: str,
    images: Sequence[dict[str, str]],
    file_hashes: dict[str, str] | None,
    bundle_sha256: str | None,
) -> dict[str, Any]:
    run_match = RUN_URL_RE.fullmatch(candidate_set['workflow_run_url'])
    if run_match is None:  # already rejected by validate_candidate_set
        raise CandidateSetAuditError('candidate workflow run URL is malformed')
    retained_files = _retained_files(file_hashes)
    if file_hashes is None:
        retained_files[0]['sha256'] = candidate_set_sha256
    return {
        'schema_version': 2,
        'schema_uri': AUDIT_SCHEMA_URI,
        'status': 'LOCAL_CONTRACT_PASS',
        'candidate_set_sha256': candidate_set_sha256,
        'candidate_bundle_sha256': bundle_sha256,
        'repository': candidate_set['repository'],
        'source_pr': candidate_set['source_pr'],
        'source_commit': candidate_set['source_commit'],
        'product_version': candidate_set['product_version'],
        'workflow_run_url': candidate_set['workflow_run_url'],
        'workflow_branch_ref': candidate_set['workflow_branch_ref'],
        'workflow_gate_commit': candidate_set['workflow_gate_commit'],
        'requested_by': candidate_set['requested_by'],
        'retained_evidence': {
            'status': (
                'FOUR_FILE_PASS' if file_hashes is not None
                else 'SET_ONLY_PASS'
            ),
            'files': retained_files,
        },
        'workflow': {
            'status': 'NOT_CHECKED',
            'run_id': int(run_match.group('run_id')),
            'event': None,
            'conclusion': None,
            'head_branch': None,
            'head_sha': None,
            'path': None,
        },
        'artifacts': {
            'status': 'NOT_CHECKED',
            'required_names': list(EXPECTED_ARTIFACTS),
            'expires_at': None,
        },
        'images': [
            {
                'ros_distro': image['ros_distro'],
                'digest': image['digest'],
                'immutable_ref': image['immutable_ref'],
                'registry_status': 'NOT_CHECKED',
                'attestation_status': 'NOT_CHECKED',
            }
            for image in images
        ],
        'findings': [],
        'authority': {
            'network_reads_performed': False,
            'temporary_artifact_copies_used': False,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def _artifact_file_report(
    report: dict[str, Any],
    artifact_name: str,
) -> dict[str, Any]:
    for item in report['retained_evidence']['files']:
        if item['artifact_name'] == artifact_name:
            return item
    raise CandidateSetAuditError(
        f'retained evidence omits artifact: {artifact_name}'
    )


def _compare_remote_artifact_contents(
    *,
    repository: str,
    run_id: int,
    artifacts_by_name: dict[str, dict[str, Any]],
    retained_paths: dict[str, Path],
    report: dict[str, Any],
    findings: list[str],
    runner: Runner,
) -> None:
    report['authority']['temporary_artifact_copies_used'] = True
    with tempfile.TemporaryDirectory(
        prefix='lidarslam-candidate-audit-'
    ) as temporary_root:
        root = Path(temporary_root)
        for artifact_name, filename, _key in EVIDENCE_FILES:
            file_report = _artifact_file_report(report, artifact_name)
            retained_path = retained_paths[filename]
            try:
                _validate_bounded_file(retained_path, filename)
                retained_hash = sha256_file(retained_path)
            except CandidateSetAuditError:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-retained-content-invalid')
                continue
            if retained_hash != file_report['sha256']:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-retained-content-changed')
                continue
            artifact = artifacts_by_name.get(artifact_name)
            if artifact is None:
                file_report['remote_content_status'] = 'FAIL'
                continue
            destination = root / artifact_name
            destination.mkdir()
            result = _run(
                [
                    'gh', 'run', 'download', str(run_id),
                    '--repo', repository,
                    '--name', artifact_name,
                    '--dir', str(destination),
                ],
                runner=runner,
            )
            if result.returncode != 0:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-download-failed')
                continue
            try:
                entries = sorted(item.name for item in destination.iterdir())
            except OSError:
                entries = []
            downloaded = destination / filename
            if entries != [filename]:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-content-layout-mismatch')
                continue
            try:
                _validate_bounded_file(downloaded, artifact_name)
                remote_hash = sha256_file(downloaded)
            except CandidateSetAuditError:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-content-invalid')
                continue
            if remote_hash != file_report['sha256']:
                file_report['remote_content_status'] = 'FAIL'
                findings.append(f'{artifact_name}-content-mismatch')
                continue
            file_report['remote_content_status'] = 'PASS'


def audit_candidate_set(
    candidate_set: dict[str, Any],
    *,
    candidate_set_sha256: str,
    remote: bool = False,
    runner: Runner = subprocess.run,
    evidence_bundle: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate local identity and optionally perform bounded remote reads."""
    if SHA256_RE.fullmatch(candidate_set_sha256) is None:
        raise CandidateSetAuditError('candidate set SHA-256 is malformed')
    images = validate_candidate_set(candidate_set)
    file_hashes = None
    bundle_hash = None
    if evidence_bundle is not None:
        if evidence_bundle.get('candidate_set') != candidate_set:
            raise CandidateSetAuditError(
                'candidate set does not match the retained evidence bundle'
            )
        file_hashes = evidence_bundle.get('file_hashes')
        bundle_hash = evidence_bundle.get('bundle_sha256')
        if not isinstance(file_hashes, dict):
            raise CandidateSetAuditError(
                'candidate evidence bundle is invalid'
            )
        canonical_bundle_hash = candidate_bundle_sha256(file_hashes)
        if (
            bundle_hash != canonical_bundle_hash
            or file_hashes['candidate-image-set.json']
            != candidate_set_sha256
        ):
            raise CandidateSetAuditError(
                'candidate evidence bundle hash does not match its files'
            )
    if remote and evidence_bundle is None:
        raise CandidateSetAuditError(
            'remote audit requires --candidate-evidence-dir with all four '
            'retained files'
        )
    if remote and (
        not isinstance(evidence_bundle.get('paths'), dict)
        or set(evidence_bundle['paths']) != set(EXPECTED_FILENAMES)
    ):
        raise CandidateSetAuditError(
            'remote audit requires canonical retained evidence paths'
        )
    report = _base_report(
        candidate_set,
        candidate_set_sha256=candidate_set_sha256,
        images=images,
        file_hashes=file_hashes,
        bundle_sha256=bundle_hash,
    )
    if not remote:
        validate_contract(report, AUDIT_SCHEMA)
        return report

    report['authority']['network_reads_performed'] = True
    findings: list[str] = []
    repository = candidate_set['repository']
    run_id = report['workflow']['run_id']

    run_result = _run(
        ['gh', 'api', f'repos/{repository}/actions/runs/{run_id}'],
        runner=runner,
    )
    run = _json_object(run_result)
    if run is None:
        findings.append('workflow-run-unavailable')
        report['workflow']['status'] = 'FAIL'
    else:
        report['workflow'].update({
            'event': run.get('event'),
            'conclusion': run.get('conclusion'),
            'head_branch': run.get('head_branch'),
            'head_sha': run.get('head_sha'),
            'path': run.get('path'),
        })
        run_ok = (
            run.get('id') == run_id
            and run.get('html_url') == candidate_set['workflow_run_url']
            and run.get('event') == 'repository_dispatch'
            and run.get('status') == 'completed'
            and run.get('conclusion') == 'success'
            and run.get('head_branch') == 'develop'
            and run.get('head_sha') == candidate_set['workflow_gate_commit']
            and run.get('path') == EXPECTED_WORKFLOW_PATH
        )
        report['workflow']['status'] = 'PASS' if run_ok else 'FAIL'
        if not run_ok:
            findings.append('workflow-run-identity-mismatch')

    artifacts_result = _run(
        [
            'gh', 'api',
            f'repos/{repository}/actions/runs/{run_id}/artifacts?per_page=100',
        ],
        runner=runner,
    )
    artifacts_payload = _json_object(artifacts_result)
    artifacts_by_name: dict[str, dict[str, Any]] = {}
    if artifacts_payload is None:
        report['artifacts']['status'] = 'FAIL'
        findings.append('workflow-artifacts-unavailable')
    else:
        artifacts = artifacts_payload.get('artifacts')
        declared_count = artifacts_payload.get('total_count')
        if (
            not isinstance(artifacts, list)
            or declared_count != len(artifacts)
            or len(artifacts) > 100
            or not all(isinstance(artifact, dict) for artifact in artifacts)
        ):
            artifacts = []
            metadata_ok = False
        else:
            metadata_ok = (
                len(artifacts) == len(EXPECTED_ARTIFACTS)
                and {artifact.get('name') for artifact in artifacts}
                == set(EXPECTED_ARTIFACTS)
            )
        by_name: dict[str, list[dict[str, Any]]] = {
            name: [] for name in EXPECTED_ARTIFACTS
        }
        for artifact in artifacts:
            if not isinstance(artifact, dict):
                continue
            name = artifact.get('name')
            if name in by_name:
                by_name[name].append(artifact)
        expires: list[str] = []
        for name in EXPECTED_ARTIFACTS:
            matches = by_name[name]
            if len(matches) != 1 or matches[0].get('expired') is not False:
                metadata_ok = False
                continue
            artifact = matches[0]
            artifact_id = artifact.get('id')
            workflow_run = artifact.get('workflow_run')
            expires_at = artifact.get('expires_at')
            if (
                not isinstance(artifact_id, int)
                or isinstance(artifact_id, bool)
                or artifact_id < 1
                or not isinstance(workflow_run, dict)
                or workflow_run.get('id') != run_id
                or workflow_run.get('head_branch') != 'develop'
                or workflow_run.get('head_sha')
                != candidate_set['workflow_gate_commit']
                or not isinstance(expires_at, str)
                or not expires_at
            ):
                metadata_ok = False
                continue
            artifacts_by_name[name] = artifact
            expires.append(expires_at)
        if metadata_ok and len(artifacts_by_name) == len(EXPECTED_ARTIFACTS):
            report['artifacts']['expires_at'] = min(expires)
            _compare_remote_artifact_contents(
                repository=repository,
                run_id=run_id,
                artifacts_by_name=artifacts_by_name,
                retained_paths=evidence_bundle['paths'],
                report=report,
                findings=findings,
                runner=runner,
            )
            content_ok = all(
                item['remote_content_status'] == 'PASS'
                for item in report['retained_evidence']['files']
            )
            report['artifacts']['status'] = 'PASS' if content_ok else 'FAIL'
        else:
            report['artifacts']['status'] = 'FAIL'
            findings.append('candidate-artifact-set-incomplete-or-expired')

    for image_report in report['images']:
        immutable_ref = image_report['immutable_ref']
        manifest_result = _run(
            [
                'docker', 'buildx', 'imagetools', 'inspect', immutable_ref,
                '--format', '{{json .Manifest}}',
            ],
            runner=runner,
        )
        manifest = _json_object(manifest_result)
        registry_ok = (
            manifest is not None
            and manifest.get('digest') == image_report['digest']
        )
        image_report['registry_status'] = 'PASS' if registry_ok else 'FAIL'
        if not registry_ok:
            findings.append(
                f"{image_report['ros_distro']}-digest-unavailable-"
                'or-mismatched'
            )

        attestation_result = _run(
            [
                'gh', 'attestation', 'verify', f'oci://{immutable_ref}',
                '-R', repository,
            ],
            runner=runner,
        )
        attestation_ok = attestation_result.returncode == 0
        image_report['attestation_status'] = (
            'PASS' if attestation_ok else 'FAIL'
        )
        if not attestation_ok:
            findings.append(
                f"{image_report['ros_distro']}-attestation-unverified"
            )

    report['findings'] = sorted(set(findings))
    report['status'] = (
        'REMOTE_AUDIT_PASS' if not report['findings']
        else 'REMOTE_AUDIT_FAIL'
    )
    validate_contract(report, AUDIT_SCHEMA)
    return report


def audit_candidate_bundle(
    evidence_bundle: dict[str, Any],
    *,
    remote: bool = False,
    runner: Runner = subprocess.run,
) -> dict[str, Any]:
    """Audit a previously loaded canonical four-file evidence bundle."""
    candidate_set = evidence_bundle['candidate_set']
    return audit_candidate_set(
        candidate_set,
        candidate_set_sha256=evidence_bundle['file_hashes'][
            'candidate-image-set.json'
        ],
        remote=remote,
        runner=runner,
        evidence_bundle=evidence_bundle,
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        '--candidate-evidence-dir',
        type=Path,
        help='directory containing exactly the four retained artifact files',
    )
    source.add_argument(
        '--candidate-image-set',
        type=Path,
        help='offline set-only compatibility check; never valid with --remote',
    )
    parser.add_argument(
        '--remote',
        action='store_true',
        help=(
            'read and byte-compare the exact workflow artifacts, GHCR '
            'digests, and GitHub attestations; never write remote state'
        ),
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Render one fail-closed local or remote candidate audit."""
    args = _parse_args(argv)
    try:
        if args.candidate_evidence_dir is not None:
            evidence_bundle = load_candidate_evidence_bundle(
                args.candidate_evidence_dir
            )
            report = audit_candidate_bundle(
                evidence_bundle,
                remote=args.remote,
            )
        else:
            if args.remote:
                raise CandidateSetAuditError(
                    '--remote requires --candidate-evidence-dir'
                )
            candidate_set = load_json_object(
                args.candidate_image_set,
                'candidate image set',
            )
            report = audit_candidate_set(
                candidate_set,
                candidate_set_sha256=sha256_file(args.candidate_image_set),
            )
    except (OSError, ValueError) as exc:
        print(f'candidate image set audit error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(report['status'])
        for finding in report['findings']:
            print(f'- {finding}')
    return 1 if report['status'] == 'REMOTE_AUDIT_FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
