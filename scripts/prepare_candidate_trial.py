#!/usr/bin/env python3
"""Download, audit, and package one exact candidate trial handoff.

The command performs bounded GitHub and registry reads, writes one new local
directory atomically, and never runs a trial or mutates remote state.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Callable, Sequence

from audit_candidate_image_set import (
    audit_candidate_bundle,
    EVIDENCE_FILES,
    EXPECTED_REPOSITORY,
    load_candidate_evidence_bundle,
    RUN_URL_RE,
)

from prepare_onboarding_matrix_packet import (
    build_candidate_packet,
    render_packet,
)

from product_schema import load_json_object, validate_contract


PREPARATION_SCHEMA = 'candidate-trial-preparation-v1.schema.json'
PREPARATION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-preparation-v1.schema.json'
)
ARTIFACTS_DIRECTORY = 'artifacts'
AUDIT_FILENAME = 'candidate-audit.json'
PACKET_JSON_FILENAME = 'observer-packet.json'
PACKET_MARKDOWN_FILENAME = 'observer-packet.md'
PREPARATION_FILENAME = 'preparation.json'
EXPECTED_ARTIFACTS = tuple(item[0] for item in EVIDENCE_FILES)
EXPECTED_FILENAMES = tuple(item[1] for item in EVIDENCE_FILES)
EXPECTED_HANDOFF_ENTRIES = (
    ARTIFACTS_DIRECTORY,
    AUDIT_FILENAME,
    PACKET_JSON_FILENAME,
    PACKET_MARKDOWN_FILENAME,
    PREPARATION_FILENAME,
)
MAX_HANDOFF_FILE_BYTES = 1024 * 1024
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateTrialPreparationError(ValueError):
    """The requested handoff cannot be prepared without guessing."""

    exit_code = 2


class CandidateTrialCheckError(CandidateTrialPreparationError):
    """A required remote or evidence check did not pass."""

    exit_code = 1


def _utc_now() -> str:
    """Return one second-resolution UTC contract timestamp."""
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _validate_request(
    workflow_run_url: str,
    output_dir: Path,
) -> tuple[int, Path]:
    """Validate identity and destination before performing a network read."""
    match = RUN_URL_RE.fullmatch(workflow_run_url)
    if match is None:
        raise CandidateTrialPreparationError(
            'workflow run URL must be the exact rsasaki0109/lidar_slam_ros2 '
            'Actions run URL'
        )
    if os.path.lexists(output_dir):
        raise CandidateTrialPreparationError(
            f'refusing to overwrite candidate handoff: {output_dir}'
        )
    parent = output_dir.parent
    try:
        parent_is_valid = parent.is_dir() and not parent.is_symlink()
    except OSError as exc:
        raise CandidateTrialPreparationError(
            f'output parent cannot be inspected: {exc}'
        ) from exc
    if not parent_is_valid:
        raise CandidateTrialPreparationError(
            'output parent must be one existing, non-symlink directory: '
            f'{parent}'
        )
    return int(match.group('run_id')), parent


def _run_download(
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
            timeout=120,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return subprocess.CompletedProcess(command, 125, '', str(exc))


def _failure_detail(result: subprocess.CompletedProcess[str]) -> str:
    detail = result.stderr.strip() or result.stdout.strip()
    if not detail:
        return f'exit {result.returncode}'
    return detail.splitlines()[0][:300]


def _download_candidate_artifacts(
    run_id: int,
    destination: Path,
    *,
    runner: Runner,
) -> None:
    """Download named single-file artifacts into one canonical directory."""
    destination.mkdir(mode=0o700)
    for artifact_name, _filename, _key in EVIDENCE_FILES:
        result = _run_download(
            [
                'gh', 'run', 'download', str(run_id),
                '--repo', EXPECTED_REPOSITORY,
                '--name', artifact_name,
                '--dir', str(destination),
            ],
            runner=runner,
        )
        if result.returncode != 0:
            raise CandidateTrialCheckError(
                f'failed to download {artifact_name}: '
                f'{_failure_detail(result)}'
            )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def _read_bounded_handoff_file(path: Path, label: str) -> bytes:
    """Read one regular, non-symlink handoff file with a fixed size bound."""
    try:
        if path.is_symlink() or not path.is_file():
            raise CandidateTrialCheckError(
                f'{label} must be one regular non-symlink file'
            )
        size = path.stat().st_size
        if size < 1 or size > MAX_HANDOFF_FILE_BYTES:
            raise CandidateTrialCheckError(
                f'{label} must be 1..{MAX_HANDOFF_FILE_BYTES} bytes'
            )
        return path.read_bytes()
    except OSError as exc:
        raise CandidateTrialCheckError(
            f'{label} cannot be inspected: {exc}'
        ) from exc


def _build_receipt(
    *,
    evidence_bundle: dict[str, Any],
    audit: dict[str, Any],
    packet: dict[str, Any],
    prepared_at: str,
) -> dict[str, Any]:
    candidate_set = evidence_bundle['candidate_set']
    receipt = {
        'schema_version': 1,
        'schema_uri': PREPARATION_SCHEMA_URI,
        'status': 'READY_FOR_OBSERVER',
        'prepared_at': prepared_at,
        'repository': candidate_set['repository'],
        'workflow_run_url': candidate_set['workflow_run_url'],
        'run_id': audit['workflow']['run_id'],
        'source_pr': candidate_set['source_pr'],
        'source_commit': candidate_set['source_commit'],
        'product_version': candidate_set['product_version'],
        'candidate_bundle_sha256': evidence_bundle['bundle_sha256'],
        'candidate_set_sha256': evidence_bundle['file_hashes'][
            'candidate-image-set.json'
        ],
        'artifact_expires_at': audit['artifacts']['expires_at'],
        'audit_status': audit['status'],
        'observer_packet_status': packet['status'],
        'artifacts': audit['retained_evidence']['files'],
        'outputs': {
            'artifacts_directory': ARTIFACTS_DIRECTORY,
            'audit_json': AUDIT_FILENAME,
            'observer_packet_json': PACKET_JSON_FILENAME,
            'observer_packet_markdown': PACKET_MARKDOWN_FILENAME,
        },
        'authority': {
            'network_reads_performed': True,
            'local_files_written': True,
            'atomic_output_publication': True,
            'trial_executed': False,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(receipt, PREPARATION_SCHEMA)
    return receipt


def load_candidate_trial_handoff(directory: Path) -> dict[str, Any]:
    """Load one exact, self-derived candidate observer handoff.

    The loader treats the retained packet and receipt as claims, not inputs.
    It rebuilds both from the canonical four artifact files and requires the
    retained remote audit, rendered Markdown, and preparation receipt to agree
    byte-for-value with that derivation.
    """
    try:
        if directory.is_symlink() or not directory.is_dir():
            raise CandidateTrialCheckError(
                'candidate handoff must be one real directory'
            )
        entries = sorted(item.name for item in directory.iterdir())
    except OSError as exc:
        raise CandidateTrialCheckError(
            f'candidate handoff cannot be inspected: {exc}'
        ) from exc
    if entries != sorted(EXPECTED_HANDOFF_ENTRIES):
        raise CandidateTrialCheckError(
            'candidate handoff must contain exactly: '
            + ', '.join(EXPECTED_HANDOFF_ENTRIES)
        )

    metadata_paths = {
        AUDIT_FILENAME: directory / AUDIT_FILENAME,
        PACKET_JSON_FILENAME: directory / PACKET_JSON_FILENAME,
        PACKET_MARKDOWN_FILENAME: directory / PACKET_MARKDOWN_FILENAME,
        PREPARATION_FILENAME: directory / PREPARATION_FILENAME,
    }
    metadata_bytes = {
        name: _read_bounded_handoff_file(path, name)
        for name, path in metadata_paths.items()
    }
    try:
        evidence_bundle = load_candidate_evidence_bundle(
            directory / ARTIFACTS_DIRECTORY
        )
        audit = load_json_object(
            metadata_paths[AUDIT_FILENAME], 'candidate handoff audit'
        )
        packet = load_json_object(
            metadata_paths[PACKET_JSON_FILENAME],
            'candidate handoff observer packet',
        )
        preparation = load_json_object(
            metadata_paths[PREPARATION_FILENAME],
            'candidate handoff preparation receipt',
        )
        validate_contract(audit, 'candidate-image-set-audit-v2.schema.json')
        validate_contract(
            packet, 'onboarding-matrix-observer-packet-v3.schema.json'
        )
        validate_contract(preparation, PREPARATION_SCHEMA)
    except ValueError as exc:
        raise CandidateTrialCheckError(
            f'candidate handoff contract is invalid: {exc}'
        ) from exc

    if audit['status'] != 'REMOTE_AUDIT_PASS':
        raise CandidateTrialCheckError(
            'candidate handoff retained audit is not REMOTE_AUDIT_PASS'
        )
    try:
        local_audit = audit_candidate_bundle(evidence_bundle)
    except ValueError as exc:
        raise CandidateTrialCheckError(
            f'candidate handoff local audit failed: {exc}'
        ) from exc
    identity_fields = (
        'candidate_set_sha256',
        'candidate_bundle_sha256',
        'repository',
        'source_pr',
        'source_commit',
        'product_version',
        'workflow_run_url',
        'workflow_branch_ref',
        'workflow_gate_commit',
        'requested_by',
    )
    if any(audit[field] != local_audit[field] for field in identity_fields):
        raise CandidateTrialCheckError(
            'candidate handoff audit identity does not match artifact bytes'
        )
    retained_identity = ('artifact_name', 'filename', 'sha256')
    retained_actual = [
        tuple(item[field] for field in retained_identity)
        for item in audit['retained_evidence']['files']
    ]
    retained_expected = [
        tuple(item[field] for field in retained_identity)
        for item in local_audit['retained_evidence']['files']
    ]
    image_identity = ('ros_distro', 'digest', 'immutable_ref')
    images_actual = [
        tuple(item[field] for field in image_identity)
        for item in audit['images']
    ]
    images_expected = [
        tuple(item[field] for field in image_identity)
        for item in local_audit['images']
    ]
    run_id = int(audit['workflow_run_url'].rsplit('/', 1)[1])
    remote_semantics_ok = (
        retained_actual == retained_expected
        and images_actual == images_expected
        and audit['workflow'] == {
            'status': 'PASS',
            'run_id': run_id,
            'event': 'repository_dispatch',
            'conclusion': 'success',
            'head_branch': 'develop',
            'head_sha': audit['workflow_gate_commit'],
            'path': '.github/workflows/candidate-image.yml',
        }
        and audit['artifacts']['required_names']
        == local_audit['artifacts']['required_names']
        and audit['authority'] == {
            'network_reads_performed': True,
            'temporary_artifact_copies_used': True,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'remote_mutations_performed': False,
        }
    )
    if not remote_semantics_ok:
        raise CandidateTrialCheckError(
            'candidate handoff retained audit semantics are inconsistent'
        )
    expected_packet = build_candidate_packet(evidence_bundle)
    if packet != expected_packet:
        raise CandidateTrialCheckError(
            'candidate handoff packet does not derive from artifact bytes'
        )
    try:
        markdown = metadata_bytes[PACKET_MARKDOWN_FILENAME].decode('utf-8')
    except UnicodeDecodeError as exc:
        raise CandidateTrialCheckError(
            'candidate handoff packet Markdown is not UTF-8'
        ) from exc
    if markdown != render_packet(expected_packet):
        raise CandidateTrialCheckError(
            'candidate handoff packet Markdown does not match packet JSON'
        )

    expected_preparation = _build_receipt(
        evidence_bundle=evidence_bundle,
        audit=audit,
        packet=packet,
        prepared_at=preparation['prepared_at'],
    )
    if preparation != expected_preparation:
        raise CandidateTrialCheckError(
            'candidate handoff preparation does not derive from retained '
            'evidence'
        )
    if preparation['workflow_run_url'] != evidence_bundle[
        'candidate_set'
    ]['workflow_run_url']:
        raise CandidateTrialCheckError(
            'candidate handoff workflow identity is inconsistent'
        )

    return {
        'directory': directory,
        'evidence_bundle': evidence_bundle,
        'audit': audit,
        'packet': packet,
        'preparation': preparation,
        'metadata_bytes': metadata_bytes,
    }


def prepare_candidate_trial(
    workflow_run_url: str,
    output_dir: Path,
    *,
    runner: Runner = subprocess.run,
    prepared_at: str | None = None,
) -> dict[str, Any]:
    """Publish one fully authenticated local candidate-trial handoff."""
    run_id, parent = _validate_request(workflow_run_url, output_dir)
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.preparing-',
        dir=parent,
    ))
    try:
        artifacts = staging / ARTIFACTS_DIRECTORY
        _download_candidate_artifacts(run_id, artifacts, runner=runner)
        try:
            evidence_bundle = load_candidate_evidence_bundle(artifacts)
        except ValueError as exc:
            raise CandidateTrialCheckError(
                f'downloaded candidate evidence is invalid: {exc}'
            ) from exc
        candidate_run_url = evidence_bundle['candidate_set'][
            'workflow_run_url'
        ]
        if candidate_run_url != workflow_run_url:
            raise CandidateTrialCheckError(
                'downloaded evidence names a different workflow run URL'
            )

        audit = audit_candidate_bundle(
            evidence_bundle,
            remote=True,
            runner=runner,
        )
        if audit['status'] != 'REMOTE_AUDIT_PASS':
            findings = ', '.join(audit['findings']) or 'unknown finding'
            raise CandidateTrialCheckError(
                f'remote candidate audit did not pass: {findings}'
            )
        packet = build_candidate_packet(evidence_bundle)
        receipt = _build_receipt(
            evidence_bundle=evidence_bundle,
            audit=audit,
            packet=packet,
            prepared_at=prepared_at or _utc_now(),
        )

        _write_json(staging / AUDIT_FILENAME, audit)
        _write_json(staging / PACKET_JSON_FILENAME, packet)
        with (staging / PACKET_MARKDOWN_FILENAME).open(
            'x', encoding='utf-8'
        ) as stream:
            stream.write(render_packet(packet))
        _write_json(staging / PREPARATION_FILENAME, receipt)
        os.replace(staging, output_dir)
        return receipt
    except CandidateTrialPreparationError:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    except (OSError, ValueError) as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialPreparationError(str(exc)) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--workflow-run-url',
        required=True,
        help=(
            'exact successful candidate-image Actions run URL; the command '
            'performs read-only GitHub, GHCR, and attestation checks'
        ),
    )
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help=(
            'new handoff directory to publish atomically; its parent must '
            'already exist and the directory is never overwritten'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='print the preparation receipt after writing the handoff',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Prepare one candidate handoff or leave the requested output absent."""
    args = _parse_args(argv)
    try:
        receipt = prepare_candidate_trial(
            args.workflow_run_url,
            args.output_dir,
        )
    except CandidateTrialPreparationError as exc:
        print(f'candidate trial preparation error: {exc}', file=sys.stderr)
        return exc.exit_code

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(f'Candidate trial handoff ready: {args.output_dir}')
        print('Remote audit: REMOTE_AUDIT_PASS')
        print(
            'Candidate bundle SHA-256: '
            f"{receipt['candidate_bundle_sha256']}"
        )
        print(
            f'Observer packet: {args.output_dir / PACKET_MARKDOWN_FILENAME}'
        )
        print('No trial or remote mutation was performed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
