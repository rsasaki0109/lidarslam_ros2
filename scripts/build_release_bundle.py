#!/usr/bin/env python3
"""Build a deterministic, manifest-backed product release bundle."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
from typing import Any, Iterable
import uuid

from product_schema import validate_contract


SCHEMA_NAME = 'release-bundle-manifest-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/release-bundle-manifest-v1.schema.json'
)
TAG_PATTERN = re.compile(r'^v[0-9]+\.[0-9]+\.[0-9]+$')
TOP_LEVEL_FILES = (
    'README.md',
    'CHANGELOG.md',
    'CONTRIBUTING.md',
    'RELEASING.md',
    'VERSION',
    'mkdocs.yml',
    'SECURITY.md',
    'SUPPORT.md',
    'CODE_OF_CONDUCT.md',
    'GOVERNANCE.md',
    'CITATION.cff',
    'Dockerfile',
)
PRODUCT_DOCS = (
    'docs/index.md',
    'docs/getting-started.md',
    'docs/getting-started-ja.md',
    'docs/usability-scorecard.md',
    'docs/product-contract.md',
    'docs/v1-readiness.md',
    'docs/golden-path-cli.md',
    'docs/cli-compatibility.md',
    'docs/operational-reliability.md',
    'docs/review-routing.md',
    'docs/real-data-e2e.md',
    'docs/external-first-map-validation.md',
    'docs/evidence/external-first-map-validations.json',
    'docs/schemas/external-first-map-validations-v1.schema.json',
    'docs/schemas/external-first-map-acceptance-v1.schema.json',
    'docs/schemas/first-map-validation-receipt-v1.schema.json',
    'docs/distribution.md',
    'docs/rosdistro-release.md',
    'docs/autoware-map-authoring.md',
    'docs/autoware-foxglove.md',
    'docs/autoware-quickstart.md',
    'docs/workflows.md',
    'docs/benchmarking.md',
    'docs/comparison.md',
    'docs/roadmap/v0.9.md',
    'docs/social/autoware_map_authoring_post_v0.9.1.md',
)
RELEASE_IMPLEMENTATION_FILES = (
    '.github/workflows/package-manager-install-upgrade.yml',
    '.github/workflows/candidate-image.yml',
    '.github/workflows/docker.yml',
    '.github/workflows/docs-site.yml',
    '.github/workflows/main.yml',
    'docker/onboarding-trial-host.Dockerfile',
    'docs/schemas/package-manager-release-readiness-v1.schema.json',
    '.github/workflows/release.yml',
    'scripts/build_release_bundle.py',
    'scripts/build_docker_launcher_asset.py',
    'scripts/check_release_bundle_reproducibility.py',
    'scripts/check_package_manager_install.py',
    'scripts/check_package_manager_release_readiness.py',
    'scripts/check_ros_apt_dependency_readiness.py',
    'scripts/check_fixture_publication.py',
    'scripts/check_issue_triage_proposal.py',
    'scripts/prepare_issue_triage_application.py',
    'scripts/check_onboarding_trial.py',
    'scripts/audit_published_fixture.py',
    'scripts/check_published_release.py',
    'scripts/check_published_onboarding_identity.py',
    'scripts/check_installed_product_cli.py',
    'scripts/check_public_docs_deployment.py',
    'scripts/check_first_map_verification_package.py',
    'scripts/check_candidate_environment.py',
    'scripts/check_product_draft_review_routing.py',
    'scripts/product_draft_review_ledger.py',
    'scripts/validate_candidate_image_request.py',
    'scripts/create_candidate_image_record.py',
    'scripts/verify_candidate_image_set.py',
    'scripts/audit_candidate_image_set.py',
    'scripts/prepare_candidate_trial.py',
    'scripts/prepare_onboarding_matrix_packet.py',
    'scripts/run_candidate_trial.py',
    'scripts/start_candidate_trial.py',
    'scripts/run_docker_onboarding_probe.py',
    'scripts/run_source_onboarding_probe.py',
    'scripts/create_release_image_record.py',
    'scripts/first_map_demo.py',
    'scripts/first_map_validator_cohort.py',
    'scripts/install_source_dependencies.sh',
    'scripts/lidarslam_cli.py',
    'scripts/measure_oci_archive.py',
    'scripts/check_external_first_map_readiness.py',
    'scripts/prepare_external_first_map_acceptance.py',
    'scripts/check_ndt_omp_release_readiness.py',
    'scripts/check_v1_readiness.py',
    'scripts/check_usability_scorecard.py',
    'scripts/prepare_usability_scorecard.py',
    'scripts/prepare_usability_scorecard_pair.py',
    'scripts/record_usability_scorecard_pair.py',
    'scripts/compare_with_glim.sh',
    'scripts/attached_storage.py',
    'scripts/download_ntu_viral_tnp01.sh',
    'scripts/download_rtk_slam_dataset.py',
    'scripts/docker_map_bag.sh',
    'scripts/first_map_validation_receipt.py',
    'scripts/create_first_map_validation_receipt.py',
    'scripts/plan_image_rollback.py',
    'scripts/glim_reference_cache.py',
    'scripts/docs_deployment_contract.py',
    'scripts/generate_docs_deployment_manifest.py',
    'scripts/generate_social_autoware_demo_video.py',
    'scripts/generate_social_autoware_map_authoring_card.py',
    'scripts/product_profiles.py',
    'scripts/product_schema.py',
    'scripts/promote_release_images.py',
    'scripts/release_channel.py',
    'scripts/run_first_map_demo.sh',
    'scripts/source_quickstart.sh',
    'scripts/support_bundle.py',
)
PRODUCT_CONFIGS = (
    'configs/real_data_e2e/driving_slam_mid360_v1.json',
)
PRODUCT_MEDIA = (
    'lidarslam/images/autoware_map_loader_proof.png',
    'lidarslam/images/dynamic_object_filter_bag6_summary.svg',
    'lidarslam/images/social_autoware_map_authoring.png',
    'lidarslam/images/social_autoware_map_authoring_demo.mp4',
    'lidarslam/images/social_autoware_map_authoring_demo.en.vtt',
    'lidarslam/images/social_autoware_map_authoring_demo.manifest.json',
)
OPTIONAL_OUTPUTS = (
    'output/benchmark_summary.md',
    'output/benchmark_summary.csv',
    'output/latest_report.html',
    'output/stress_validation_report_20260325.md',
    'output/v2_beta_readiness_20260324.md',
)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _recursive_files(root: Path, relative_dir: str) -> Iterable[str]:
    directory = root / relative_dir
    if not directory.is_dir():
        raise ValueError(
            f'required release directory is missing: {relative_dir}'
        )
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'release bundle refuses symlink: {path}')
        if path.is_file():
            yield path.relative_to(root).as_posix()


def release_bundle_paths(root: Path, tag: str) -> list[str]:
    """Return the sorted, duplicate-free curated release file inventory."""
    required = [
        *TOP_LEVEL_FILES,
        *PRODUCT_DOCS,
        *RELEASE_IMPLEMENTATION_FILES,
        *PRODUCT_CONFIGS,
        *PRODUCT_MEDIA,
        f'docs/releases/{tag}.md',
    ]
    paths = set(required)
    for relative_dir in (
        'docs/assets',
        'docs/contracts',
        'docs/evidence',
        'docs/schemas',
    ):
        paths.update(_recursive_files(root, relative_dir))
    paths.update(
        relative for relative in OPTIONAL_OUTPUTS
        if (root / relative).is_file()
    )

    for relative in sorted(paths):
        path = root / relative
        if path.is_symlink():
            raise ValueError(f'release bundle refuses symlink: {relative}')
        if not path.is_file():
            raise ValueError(f'required release file is missing: {relative}')
    return sorted(paths)


def build_manifest(
    root: Path,
    *,
    tag: str,
    git_commit: str,
    paths: Iterable[str],
) -> dict[str, Any]:
    """Build the public checksum inventory embedded in every bundle."""
    version = (root / 'VERSION').read_text(encoding='utf-8').strip()
    if tag != f'v{version}' or TAG_PATTERN.fullmatch(tag) is None:
        raise ValueError(
            f'release tag {tag!r} does not match VERSION {version!r}'
        )
    files = []
    for relative in paths:
        payload = (root / relative).read_bytes()
        files.append({
            'path': relative,
            'size_bytes': len(payload),
            'sha256': _sha256_bytes(payload),
        })
    manifest = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': 'PASS',
        'tag': tag,
        'product_version': version,
        'git_commit': git_commit,
        'files': files,
    }
    validate_contract(manifest, SCHEMA_NAME)
    return manifest


def _tar_info(
    name: str,
    payload: bytes,
    *,
    mode: int = 0o644,
) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name=name)
    info.size = len(payload)
    info.mode = mode
    info.mtime = 0
    info.uid = 0
    info.gid = 0
    info.uname = 'root'
    info.gname = 'root'
    return info


def build_release_bundle(
    root: Path,
    output: Path,
    *,
    tag: str,
    git_commit: str,
) -> dict[str, Any]:
    """Write one deterministic bundle without mutating source files."""
    if output.exists():
        raise ValueError(f'refusing to overwrite release bundle: {output}')
    if not output.parent.is_dir():
        raise ValueError(
            f'release bundle parent does not exist: {output.parent}'
        )
    version = (root / 'VERSION').read_text(encoding='utf-8').strip()
    if tag != f'v{version}' or TAG_PATTERN.fullmatch(tag) is None:
        raise ValueError(
            f'release tag {tag!r} does not match VERSION {version!r}'
        )
    paths = release_bundle_paths(root, tag)
    manifest = build_manifest(
        root,
        tag=tag,
        git_commit=git_commit,
        paths=paths,
    )
    manifest_payload = (
        json.dumps(manifest, indent=2, sort_keys=True) + '\n'
    ).encode()
    temp = output.with_name(f'.{output.name}.{uuid.uuid4().hex}.tmp')
    try:
        with temp.open('xb') as raw:
            with gzip.GzipFile(
                filename='',
                mode='wb',
                fileobj=raw,
                mtime=0,
            ) as compressed:
                with tarfile.open(
                    fileobj=compressed,
                    mode='w',
                    format=tarfile.GNU_FORMAT,
                ) as archive:
                    for relative in paths:
                        payload = (root / relative).read_bytes()
                        archive.addfile(
                            _tar_info(f'release_bundle/{relative}', payload),
                            io.BytesIO(payload),
                        )
                    archive.addfile(
                        _tar_info(
                            'release_bundle/release-bundle-manifest-v1.json',
                            manifest_payload,
                        ),
                        io.BytesIO(manifest_payload),
                    )
        temp.replace(output)
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    return manifest


def parse_args() -> argparse.Namespace:
    """Parse release-bundle command-line arguments."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_RELEASE_COMMAND'),
        description=(
            'Build a deterministic, checksummed product release bundle.'
        ),
    )
    parser.add_argument('--tag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--candidate',
        action='store_true',
        help=(
            'Allow a not-yet-created tag; an existing tag must still name '
            'HEAD.'
        ),
    )
    return parser.parse_args()


def main() -> int:
    """Build a release bundle for the checked-out commit."""
    args = parse_args()
    root = Path(__file__).resolve().parent.parent
    output = Path(args.output).expanduser().resolve()
    try:
        status = subprocess.run(
            ['git', 'status', '--porcelain'],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
        )
        if status.stdout:
            raise ValueError('release bundle requires a clean source worktree')
        git_commit = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        tagged_commit = subprocess.run(
            [
                'git',
                'rev-parse',
                '--verify',
                f'refs/tags/{args.tag}^{{commit}}',
            ],
            cwd=root,
            check=False,
            capture_output=True,
            text=True,
        )
        if tagged_commit.returncode == 0:
            observed = tagged_commit.stdout.strip()
            if observed != git_commit:
                raise ValueError(
                    f'tag {args.tag} already names {observed}, not HEAD '
                    f'{git_commit}'
                )
        elif not args.candidate:
            raise ValueError(
                f'tag {args.tag} does not exist; use --candidate only for '
                'a pre-tag rehearsal'
            )
        manifest = build_release_bundle(
            root,
            output,
            tag=args.tag,
            git_commit=git_commit,
        )
    except (OSError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    print(json.dumps({
        'status': manifest['status'],
        'bundle': str(output),
        'files': len(manifest['files']),
        'git_commit': manifest['git_commit'],
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
