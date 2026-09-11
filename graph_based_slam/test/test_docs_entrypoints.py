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

"""Smoke tests for top-level docs entry points."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

import jsonschema
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
README_PATH = REPO_ROOT / 'README.md'
CONTRIBUTING_PATH = REPO_ROOT / 'CONTRIBUTING.md'
VERSION_PATH = REPO_ROOT / 'VERSION'
CHANGELOG_PATH = REPO_ROOT / 'CHANGELOG.md'
RELEASING_PATH = REPO_ROOT / 'RELEASING.md'
SECURITY_PATH = REPO_ROOT / 'SECURITY.md'
SUPPORT_PATH = REPO_ROOT / 'SUPPORT.md'
CODE_OF_CONDUCT_PATH = REPO_ROOT / 'CODE_OF_CONDUCT.md'
GOVERNANCE_PATH = REPO_ROOT / 'GOVERNANCE.md'
CITATION_PATH = REPO_ROOT / 'CITATION.cff'
MKDOCS_CONFIG_PATH = REPO_ROOT / 'mkdocs.yml'
GITIGNORE_PATH = REPO_ROOT / '.gitignore'
DOCS_INDEX_PATH = REPO_ROOT / 'docs' / 'index.md'
GETTING_STARTED = REPO_ROOT / 'docs' / 'getting-started.md'
GETTING_STARTED_JA = REPO_ROOT / 'docs' / 'getting-started-ja.md'
USABILITY_SCORECARD_DOC = REPO_ROOT / 'docs' / 'usability-scorecard.md'
DOCS_ASSETS_DIR = REPO_ROOT / 'docs' / 'assets'
DOCS_EXTRA_CSS_PATH = DOCS_ASSETS_DIR / 'stylesheets' / 'extra.css'
DOCS_AUTOWARE_PROOF_SITE_IMAGE_PATH = DOCS_ASSETS_DIR / 'images' / 'autoware_map_loader_proof.png'
DOCS_DYNAMIC_FILTER_SITE_IMAGE_PATH = (
    DOCS_ASSETS_DIR / 'images' / 'dynamic_object_filter_bag6_summary.svg'
)
AUTOWARE_QUICKSTART = REPO_ROOT / 'docs' / 'autoware-quickstart.md'
AUTOWARE_MAP_AUTHORING = REPO_ROOT / 'docs' / 'autoware-map-authoring.md'
AUTOWARE_FOXGLOVE = REPO_ROOT / 'docs' / 'autoware-foxglove.md'
WORKFLOWS_DOC = REPO_ROOT / 'docs' / 'workflows.md'
ONBOARDING_TRIAL_EXECUTION_DOC = (
    REPO_ROOT / 'docs' / 'onboarding-trial-execution.md'
)
ONBOARDING_TRIALS_DOC = REPO_ROOT / 'docs' / 'onboarding-trials.md'
ONBOARDING_MEASUREMENT_SCRIPT = (
    REPO_ROOT / 'scripts' / 'complete_onboarding_measurements.py'
)
ONBOARDING_MEASUREMENT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'onboarding-measurement-supplement-v1.schema.json'
)
SOURCE_DEPENDENCIES_SCRIPT = (
    REPO_ROOT / 'scripts' / 'install_source_dependencies.sh'
)
SOURCE_QUICKSTART_SCRIPT = REPO_ROOT / 'scripts' / 'source_quickstart.sh'
DOCKER_MAP_BAG_SCRIPT = REPO_ROOT / 'scripts' / 'docker_map_bag.sh'
SOURCE_ONBOARDING_PROBE = (
    REPO_ROOT / 'scripts' / 'run_source_onboarding_probe.py'
)
BENCHMARKING_DOC = REPO_ROOT / 'docs' / 'benchmarking.md'
COMPARISON_DOC = REPO_ROOT / 'docs' / 'comparison.md'
PRODUCT_CONTRACT_DOC = REPO_ROOT / 'docs' / 'product-contract.md'
V1_READINESS_DOC = REPO_ROOT / 'docs' / 'v1-readiness.md'
V1_READINESS_CONTRACT = (
    REPO_ROOT / 'docs' / 'contracts' / 'v1-readiness.json'
)
V1_READINESS_CONTRACT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-contract-v1.schema.json'
)
V1_READINESS_REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-report-v1.schema.json'
)
V1_READINESS_SCRIPT = REPO_ROOT / 'scripts' / 'check_v1_readiness.py'
GOLDEN_PATH_CLI_DOC = REPO_ROOT / 'docs' / 'golden-path-cli.md'
CLI_COMPATIBILITY_DOC = REPO_ROOT / 'docs' / 'cli-compatibility.md'
CLI_V1_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
DISTRIBUTION_DOC = REPO_ROOT / 'docs' / 'distribution.md'
ROSDISTRO_RELEASE_DOC = REPO_ROOT / 'docs' / 'rosdistro-release.md'
MAIN_CI_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'main.yml'
OFFICIAL_RKO_COMPATIBILITY_WORKFLOW = (
    REPO_ROOT
    / '.github'
    / 'workflows'
    / 'official-rko-binary-compatibility.yml'
)
OPERATIONAL_RELIABILITY_DOC = REPO_ROOT / 'docs' / 'operational-reliability.md'
BOUNDED_FILESYSTEM_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'bounded-filesystem-exhaustion-v1.schema.json'
)
BOUNDED_FILESYSTEM_WORKFLOW = (
    REPO_ROOT
    / '.github'
    / 'workflows'
    / 'bounded-filesystem-exhaustion.yml'
)
SOAK_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'real-data-soak-2026-07-28.md'
)
SOAK_EVIDENCE_JSON = (
    REPO_ROOT / 'docs' / 'evidence' / 'real-data-soak-2026-07-28.json'
)
OFFICIAL_RKO_EVIDENCE_JSON = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'official-rko-binary-compatibility-2026-07-29.json'
)
OFFICIAL_RKO_EVIDENCE_DOC = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'official-rko-binary-compatibility-2026-07-29.md'
)
ROS_APT_READINESS_20260812 = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'ros-apt-dependency-readiness-2026-08-12.json'
)
DOCKER_FIRST_MAP_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'docker-first-map-2026-07-28.md'
)
EXTERNAL_FIRST_MAP_DOC = (
    REPO_ROOT / 'docs' / 'external-first-map-validation.md'
)
EXTERNAL_FIRST_MAP_LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
EXTERNAL_FIRST_MAP_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)
EXTERNAL_FIRST_MAP_SCRIPT = (
    REPO_ROOT / 'scripts' / 'check_external_first_map_readiness.py'
)
CLI_V1_INSTALL_EVIDENCE_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'cli-v1-install-2026-07-28.md'
)
TIMESTAMP_ORDER_EVIDENCE_DOC = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'timestamp-order-preflight-2026-07-29.md'
)
REAL_DATA_E2E_DOC = REPO_ROOT / 'docs' / 'real-data-e2e.md'
PREFLIGHT_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'preflight-v6.schema.json'
PUBLIC_DOCTOR_EVIDENCE_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'public-doctor-evidence-v1.schema.json'
)
DIAGNOSIS_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'
MAP_SESSION_RECOVERY_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'map-session-recovery-v1.schema.json'
)
RUN_MANIFEST_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v1.schema.json'
)
RUN_MANIFEST_V2_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
)
RELEASE_IMAGE_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'release-image-v1.schema.json'
)
CANDIDATE_IMAGE_REQUEST_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'candidate-image-request-v1.schema.json'
)
CANDIDATE_IMAGE_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'candidate-image-v1.schema.json'
)
CANDIDATE_IMAGE_SET_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'candidate-image-set-v1.schema.json'
)
CANDIDATE_IMAGE_SET_AUDIT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'candidate-image-set-audit-v1.schema.json'
)
CANDIDATE_IMAGE_SET_AUDIT_V2_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'candidate-image-set-audit-v2.schema.json'
)
ONBOARDING_OBSERVER_PACKET_V2_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'onboarding-matrix-observer-packet-v2.schema.json'
)
ONBOARDING_OBSERVER_PACKET_V3_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'onboarding-matrix-observer-packet-v3.schema.json'
)
CANDIDATE_IMAGE_SET_AUDIT_SCRIPT = (
    REPO_ROOT / 'scripts' / 'audit_candidate_image_set.py'
)
ROLLBACK_PLAN_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'rollback-plan-v1.schema.json'
)
RELEASE_BUNDLE_MANIFEST_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'release-bundle-manifest-v1.schema.json'
)
RELEASE_PROMOTION_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'release-promotion-v1.schema.json'
)
V09_ROADMAP_DOC = REPO_ROOT / 'docs' / 'roadmap' / 'v0.9.md'
SOCIAL_POST_DOC = (
    REPO_ROOT / 'docs' / 'social' / 'autoware_map_authoring_post_v0.9.1.md'
)
ISSUE_TEMPLATE_DIR = REPO_ROOT / '.github' / 'ISSUE_TEMPLATE'
PUBLIC_AUTOWARE_ENTRYPOINT = REPO_ROOT / 'scripts' / 'run_autoware_quickstart.sh'
RELEASE_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'release.yml'
CANDIDATE_IMAGE_WORKFLOW = (
    REPO_ROOT / '.github' / 'workflows' / 'candidate-image.yml'
)
RELEASE_BUNDLE_SCRIPT = REPO_ROOT / 'scripts' / 'build_release_bundle.py'
RELEASE_PROMOTION_SCRIPT = REPO_ROOT / 'scripts' / 'promote_release_images.py'
DOCKER_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'docker.yml'
DOCS_SITE_WORKFLOW = REPO_ROOT / '.github' / 'workflows' / 'docs-site.yml'
README_LOOP_IMAGE_PATH = REPO_ROOT / 'lidarslam' / 'images' / 'mid360_loop_closure_zoom.png'
README_AUTOWARE_PROOF_IMAGE_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'autoware_map_loader_proof.png'
)
README_DYNAMIC_FILTER_IMAGE_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'dynamic_object_filter_bag6_summary.svg'
)
SOCIAL_CARD_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'social_autoware_map_authoring.png'
)
SOCIAL_DEMO_VIDEO_PATH = (
    REPO_ROOT / 'lidarslam' / 'images' / 'social_autoware_map_authoring_demo.mp4'
)
SOCIAL_DEMO_CAPTIONS_PATH = (
    REPO_ROOT
    / 'lidarslam'
    / 'images'
    / 'social_autoware_map_authoring_demo.en.vtt'
)
SOCIAL_DEMO_MANIFEST_PATH = (
    REPO_ROOT
    / 'lidarslam'
    / 'images'
    / 'social_autoware_map_authoring_demo.manifest.json'
)


def test_docs_exist_and_are_linked_from_readme():
    """README should link to the main adoption-oriented docs."""
    readme = README_PATH.read_text(encoding='utf-8')
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    release_notes_path = REPO_ROOT / 'docs' / 'releases' / f'v{version}.md'

    assert CONTRIBUTING_PATH.is_file()
    assert VERSION_PATH.is_file()
    assert CHANGELOG_PATH.is_file()
    assert RELEASING_PATH.is_file()
    assert SECURITY_PATH.is_file()
    assert SUPPORT_PATH.is_file()
    assert CODE_OF_CONDUCT_PATH.is_file()
    assert GOVERNANCE_PATH.is_file()
    assert CITATION_PATH.is_file()
    assert MKDOCS_CONFIG_PATH.is_file()
    assert DOCS_INDEX_PATH.is_file()
    assert GETTING_STARTED.is_file()
    assert USABILITY_SCORECARD_DOC.is_file()
    assert DOCS_ASSETS_DIR.is_dir()
    assert DOCS_EXTRA_CSS_PATH.is_file()
    assert DOCS_AUTOWARE_PROOF_SITE_IMAGE_PATH.is_file()
    assert DOCS_DYNAMIC_FILTER_SITE_IMAGE_PATH.is_file()
    assert AUTOWARE_QUICKSTART.is_file()
    assert AUTOWARE_MAP_AUTHORING.is_file()
    assert AUTOWARE_FOXGLOVE.is_file()
    assert WORKFLOWS_DOC.is_file()
    assert BENCHMARKING_DOC.is_file()
    assert COMPARISON_DOC.is_file()
    assert PRODUCT_CONTRACT_DOC.is_file()
    assert V1_READINESS_DOC.is_file()
    assert V1_READINESS_CONTRACT.is_file()
    assert V1_READINESS_CONTRACT_SCHEMA.is_file()
    assert V1_READINESS_REPORT_SCHEMA.is_file()
    assert V1_READINESS_SCRIPT.is_file()
    assert GOLDEN_PATH_CLI_DOC.is_file()
    assert CLI_COMPATIBILITY_DOC.is_file()
    assert CLI_V1_CONTRACT.is_file()
    assert DISTRIBUTION_DOC.is_file()
    assert ROSDISTRO_RELEASE_DOC.is_file()
    assert OFFICIAL_RKO_COMPATIBILITY_WORKFLOW.is_file()
    assert OFFICIAL_RKO_EVIDENCE_JSON.is_file()
    assert OFFICIAL_RKO_EVIDENCE_DOC.is_file()
    assert ROS_APT_READINESS_20260812.is_file()
    assert OPERATIONAL_RELIABILITY_DOC.is_file()
    assert BOUNDED_FILESYSTEM_SCHEMA.is_file()
    assert BOUNDED_FILESYSTEM_WORKFLOW.is_file()
    assert SOAK_EVIDENCE_DOC.is_file()
    assert SOAK_EVIDENCE_JSON.is_file()
    assert EXTERNAL_FIRST_MAP_DOC.is_file()
    assert EXTERNAL_FIRST_MAP_LEDGER.is_file()
    assert EXTERNAL_FIRST_MAP_SCHEMA.is_file()
    assert EXTERNAL_FIRST_MAP_SCRIPT.is_file()
    assert CLI_V1_INSTALL_EVIDENCE_DOC.is_file()
    assert REAL_DATA_E2E_DOC.is_file()
    assert PREFLIGHT_SCHEMA.is_file()
    assert PUBLIC_DOCTOR_EVIDENCE_SCHEMA.is_file()
    assert DIAGNOSIS_SCHEMA.is_file()
    assert MAP_SESSION_RECOVERY_SCHEMA.is_file()
    assert RUN_MANIFEST_SCHEMA.is_file()
    assert RUN_MANIFEST_V2_SCHEMA.is_file()
    assert RELEASE_IMAGE_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_REQUEST_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_SET_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_SET_AUDIT_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_SET_AUDIT_V2_SCHEMA.is_file()
    assert ONBOARDING_OBSERVER_PACKET_V2_SCHEMA.is_file()
    assert ONBOARDING_OBSERVER_PACKET_V3_SCHEMA.is_file()
    assert CANDIDATE_IMAGE_SET_AUDIT_SCRIPT.is_file()
    assert ROLLBACK_PLAN_SCHEMA.is_file()
    assert RELEASE_BUNDLE_MANIFEST_SCHEMA.is_file()
    assert RELEASE_PROMOTION_SCHEMA.is_file()
    assert RELEASE_BUNDLE_SCRIPT.is_file()
    assert RELEASE_PROMOTION_SCRIPT.is_file()
    assert V09_ROADMAP_DOC.is_file()
    assert SOCIAL_POST_DOC.is_file()
    assert DOCKER_WORKFLOW.is_file()
    assert CANDIDATE_IMAGE_WORKFLOW.is_file()
    assert DOCS_SITE_WORKFLOW.is_file()
    assert README_LOOP_IMAGE_PATH.is_file()
    assert README_AUTOWARE_PROOF_IMAGE_PATH.is_file()
    assert README_DYNAMIC_FILTER_IMAGE_PATH.is_file()
    assert SOCIAL_CARD_PATH.is_file()
    assert SOCIAL_DEMO_VIDEO_PATH.is_file()
    assert SOCIAL_DEMO_CAPTIONS_PATH.is_file()
    assert SOCIAL_DEMO_MANIFEST_PATH.is_file()
    assert release_notes_path.is_file()
    assert '(CONTRIBUTING.md)' in readme
    assert '(CHANGELOG.md)' in readme
    assert '(RELEASING.md)' in readme
    assert '(SECURITY.md)' in readme
    assert '(SUPPORT.md)' in readme
    assert '(GOVERNANCE.md)' in readme
    assert '(docs/product-contract.md)' in readme
    assert '(docs/v1-readiness.md)' in readme
    assert '(docs/external-first-map-validation.md)' in readme
    assert '(docs/distribution.md)' in readme
    assert '(docs/roadmap/v0.9.md)' in readme
    assert '(docs/getting-started.md)' in readme
    assert '(docs/autoware-map-authoring.md)' in readme
    assert '(docs/autoware-quickstart.md)' in readme
    assert '(docs/autoware-foxglove.md)' in readme
    assert '(docs/workflows.md)' in readme
    assert '(docs/comparison.md)' in readme
    assert '(docs/benchmarking.md)' in readme
    assert 'python3 -m mkdocs serve' in readme
    assert 'lidarslam-map run' in readme
    assert '(lidarslam/images/autoware_map_loader_proof.png)' in readme
    assert 'git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git' in readme
    assert 'bash scripts/source_quickstart.sh' in readme
    assert 'Run `lidarslam-map` with no arguments' in readme
    assert 'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble' in readme
    assert 'v0.9.1` release candidate is not published yet' in readme
    # The required-topics table and the dynamic-object-filter figure moved to
    # docs/workflows.md so the README stays narrow; keep the assets on disk
    # (asserted above) and verify the README still routes readers to those docs.
    assert f'(docs/releases/v{version}.md)' in readme
    assert len(readme.splitlines()) <= 220


def test_readme_chooser_routes_each_goal_to_one_safe_first_step():
    """Repository visitors should choose a bounded path before details."""
    readme = README_PATH.read_text(encoding='utf-8')
    chooser = readme.split('### Choose your shortest path', 1)[1].split(
        '### Try it with Docker', 1
    )[0]

    assert '| Goal | Start here | Safety and cost boundary |' in chooser
    assert '**Default if unsure:**' in chooser
    assert '[Docker demo](#try-it-with-docker-one-command-no-build)' in chooser
    assert 'Stable `v0.9.0-humble`' in chooser
    assert 'host writes stay in `./lidarslam_output`' in chooser
    assert 'lidarslam-map doctor /path/to/rosbag2' in chooser
    assert 'Read-only diagnosis first' in chooser
    assert 'lidarslam-map start /path/to/rosbag2' in chooser
    assert '[Source quickstart](#build--verified-demo-from-source-one-helper)' in (
        chooser
    )
    assert 'bash scripts/source_quickstart.sh --dry-run' in chooser
    assert 'Candidate `v0.9.1`' in chooser
    assert 'needs ROS 2, 8 GiB, and roughly 30 minutes' in chooser


def test_readme_names_the_fixed_demo_receipt_handoff():
    """The repository front door must expose the receipt-only next step."""
    readme = README_PATH.read_text(encoding='utf-8')
    assert 'lidarslam-map report /path/to/output/mid360_demo --json' in readme
    assert '[first-map validation guide](docs/external-first-map-validation.md)' in (
        readme
    )


def test_getting_started_chooser_limits_the_first_decision_to_three_goals():
    """Canonical onboarding should hide advanced actions from first choice."""
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    docs_index = DOCS_INDEX_PATH.read_text(encoding='utf-8')
    chooser = getting_started.split('## Choose A Path', 1)[1].split(
        '<details markdown="1">', 1
    )[0]
    continuing = getting_started.split(
        '<summary>Already installed, or continuing after your first map?'
        '</summary>',
        1,
    )[1].split('</details>', 1)[0]

    chooser_rows = [
        line for line in chooser.splitlines() if line.startswith('|')
    ]
    assert len(chooser_rows) == 5  # header, separator, and exactly 3 goals
    assert '| Goal | First safe action | Safety and cost boundary |' in chooser
    assert '**Default if unsure:**' in chooser
    assert 'Stable `v0.9.0-humble`' in chooser
    assert 'lidarslam-map doctor /path/to/rosbag2' in chooser
    assert 'Diagnosis uses no network and writes no files' in chooser
    assert 'bash scripts/source_quickstart.sh --dry-run' in chooser
    assert 'Candidate `v0.9.1`' in chooser
    assert 'needs ROS 2, 8 GiB, and roughly 30 minutes' in chooser

    for advanced_action in (
        'lidarslam-map demo',
        'bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2',
        'lidarslam-map start /path/to/rosbag2 --editable',
        'lidarslam-map merge output/day1 output/day2',
        '--lidar-to-base ... --imu-to-base ...',
        'lidarslam-map run /path/to/rosbag2 --output-dir',
    ):
        assert advanced_action not in chooser
        assert advanced_action in continuing

    assert '[v0.9.0 stable release](releases/v0.9.0.md)' in docs_index
    assert 'v0.9.0 stable release candidate' not in docs_index


def test_getting_started_keeps_legacy_source_anchor_for_cohort_issue():
    """The tracked validator issue must not land on a dead source section."""
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    assert '<a id="1-build-the-workspace"></a>' in getting_started
    assert '## 1. Install And Build From Source' in getting_started


def test_first_map_docs_expose_stable_image_receipt_fallback():
    """The public v0.9.0 image needs a bounded receipt-only escape hatch."""
    documents = [
        GETTING_STARTED.read_text(encoding='utf-8'),
        GETTING_STARTED_JA.read_text(encoding='utf-8'),
        EXTERNAL_FIRST_MAP_DOC.read_text(encoding='utf-8'),
    ]

    for document in documents:
        assert 'create_first_map_validation_receipt.py' in document
        assert 'python3 scripts/create_first_map_validation_receipt.py' in document
        assert '--network none' in document
        assert 'lidarslam_output:/output:ro' in document
        assert 'ros2 pkg prefix lidarslam' in document
        assert 'docker image inspect' in document
        assert 'v0.9.0-humble' in document

    english = ' '.join(documents[0].split())
    assert 'Review the printed PASS summary' in english
    assert 'attach only the already-generated JSON receipt' in english


def test_public_schemas_support_ros_distro_jsonschema():
    """Public schemas must work with the dependency shipped by ROS distros."""
    schemas = sorted((REPO_ROOT / 'docs' / 'schemas').glob('*.json'))
    assert schemas

    for path in schemas:
        schema = json.loads(path.read_text(encoding='utf-8'))
        assert schema['$schema'] == 'http://json-schema.org/draft-07/schema#'
        assert '$defs' not in schema
        jsonschema.Draft7Validator.check_schema(schema)

    for path in (REPO_ROOT / 'scripts').glob('*.py'):
        source = path.read_text(encoding='utf-8')
        assert 'Draft202012Validator' not in source
        assert 'version=9' not in source


def test_all_tracked_shell_scripts_parse_with_bash():
    """A documented shell entry point must never be committed unparsable."""
    inventory = subprocess.run(
        ['git', 'ls-files', '-z', '*.sh'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
    )
    assert inventory.returncode == 0, inventory.stderr.decode()
    paths = [
        path.decode()
        for path in inventory.stdout.split(b'\0')
        if path
    ]
    assert 'scripts/compare_with_glim.sh' in paths

    failures = []
    for path in paths:
        result = subprocess.run(
            ['bash', '-n', path],
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            failures.append(f'{path}: {result.stderr.strip()}')
    assert not failures, '\n'.join(failures)


def test_docs_reference_existing_entrypoint_scripts():
    """Every documented entrypoint script should exist in the repo."""
    scripts = [
        PUBLIC_AUTOWARE_ENTRYPOINT,
        REPO_ROOT / 'scripts' / 'download_ntu_viral_tnp01.sh',
        REPO_ROOT / 'scripts' / 'run_first_map_demo.sh',
        REPO_ROOT / 'scripts' / 'run_docker_demo.sh',
        REPO_ROOT / 'scripts' / 'run_default_ci_checks.sh',
        REPO_ROOT / 'scripts' / 'run_product_python_tests.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh',
        REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware.sh',
        REPO_ROOT / 'scripts' / 'prepare_autoware_map_from_graph_slam.sh',
        REPO_ROOT / 'scripts' / 'create_map_authoring_submission_bundle.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_pointcloud_map_viewer_docker.sh',
        REPO_ROOT / 'scripts' / 'prepare_foxglove_bridge_prefix.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_pointcloud_map_foxglove.sh',
        REPO_ROOT / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_graph_benchmark.sh',
        REPO_ROOT / 'scripts' / 'run_radarless_tunnel_ab.sh',
        REPO_ROOT / 'scripts' / 'run_rko_lio_mid360_crossval_benchmark.sh',
        REPO_ROOT / 'scripts' / 'export_mid360_robot_3d_map_preview.py',
        REPO_ROOT / 'scripts' / 'analyze_mid360_robot_public_loop_cloud.py',
        REPO_ROOT / 'scripts' / 'plan_mid360_robot_public_loop_segment_reset.py',
        REPO_ROOT / 'scripts' / 'analyze_mid360_robot_public_segment_map_cloud_alignment.py',
        REPO_ROOT / 'scripts' / 'run_mid360_robot_public_completion_gate.py',
        REPO_ROOT / 'scripts' / 'run_mid360_robot_public_continuous_relocalization_gate.py',
        REPO_ROOT / 'scripts' / 'merge_mid360_robot_public_split_bags.py',
        REPO_ROOT / 'scripts' / 'run_release_readiness_checks.sh',
        ONBOARDING_MEASUREMENT_SCRIPT,
        REPO_ROOT / 'scripts' / 'benchmark_summary.py',
        REPO_ROOT / 'scripts' / 'generate_html_report.py',
        REPO_ROOT / 'scripts' / 'generate_v2_beta_readiness_report.py',
        REPO_ROOT / 'scripts' / 'generate_map_authoring_report.py',
        REPO_ROOT / 'scripts' / 'generate_stress_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_readme_dynamic_filter_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_autoware_proof_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_large_loop_map_figure.py',
        REPO_ROOT / 'scripts' / 'generate_readme_loop_zoom_figure.py',
        REPO_ROOT / 'scripts' / 'generate_social_autoware_map_authoring_card.py',
        REPO_ROOT / 'scripts' / 'generate_social_autoware_demo_video.py',
        REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py',
        REPO_ROOT / 'scripts' / 'generate_sample_benchmark_metrics.py',
        REPO_ROOT / 'scripts' / 'inspect_navsatfix_covariance.py',
        REPO_ROOT / 'scripts' / 'inspect_applanix_gsof50_quality.py',
        REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_navsatfix_bag.py',
        REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_imu_bag.py',
        REPO_ROOT / 'scripts' / 'extract_applanix_gsof49_reference.py',
        REPO_ROOT / 'scripts' / 'extract_static_transform_from_bag.py',
        REPO_ROOT / 'scripts' / 'prepare_velodyne_pointcloud_overlay.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_gnss_smoke.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_smoke.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_applanix_velodyne_gnss_benchmark.sh',
        REPO_ROOT / 'scripts' / 'run_open_data_classic_path_benchmark_suite.sh',
        REPO_ROOT / 'scripts' / 'generate_odom_prior_validation_report.py',
        REPO_ROOT / 'scripts' / 'run_open_data_packet_imu_deskew_validation_matrix.sh',
        REPO_ROOT / 'scripts' / 'run_dynamic_object_filter_benchmark.sh',
        REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_exploration_closeout_report.py',
        REPO_ROOT / 'scripts' / 'run_place_recognition_benchmark.sh',
        REPO_ROOT / 'scripts' / 'generate_classic_path_report.py',
        REPO_ROOT / 'scripts' / 'generate_place_recognition_report.py',
        REPO_ROOT / 'scripts' / 'generate_packet_imu_deskew_validation_report.py',
        REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_report.py',
        REPO_ROOT / 'scripts' / 'preflight_autoware_map_bag.py',
        REPO_ROOT / 'scripts' / 'run_autoware_map_beginner.sh',
        REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py',
        REPO_ROOT / 'scripts' / 'diagnose_autoware_map_run.py',
        REPO_ROOT / 'scripts' / 'verify_autoware_map.py',
    ]
    for path in scripts:
        assert path.is_file(), path


def test_contributing_and_issue_templates_exist():
    """Community entry points should cover support and structured reports."""
    contributing = CONTRIBUTING_PATH.read_text(encoding='utf-8')

    assert ISSUE_TEMPLATE_DIR.is_dir()
    assert (ISSUE_TEMPLATE_DIR / 'config.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'benchmark-report.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'autoware-pointcloud-map.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'bug-report.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'feature-request.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'sensor-support.yml').is_file()
    assert (ISSUE_TEMPLATE_DIR / 'first-map-validation.yml').is_file()
    assert 'Benchmark Result Submissions' in contributing
    assert 'Autoware Naming And Trademark Guidance' in contributing
    assert 'Autoware-compatible pointcloud map' in contributing
    assert 'official Autoware' in contributing
    assert 'endorsed by the Autoware Foundation' in contributing
    assert 'run_release_readiness_checks.sh' in contributing
    assert 'run_autoware_quickstart.sh' in contributing
    assert 'run_product_python_tests.sh' in contributing
    assert 'The four official beginner-facing product workflows are:' in (
        contributing
    )
    assert '`lidarslam-map demo`' in contributing
    assert '`lidarslam-map start <rosbag2_dir>`' in contributing
    assert '`lidarslam-map sessions`' in contributing
    assert '`lidarslam-map support <session_bundle>`' in contributing
    assert 'run `lidarslam-map` without' in contributing
    assert 'The three official beginner-facing product entrypoints' not in (
        contributing
    )
    assert 'own-bag wrapper: `scripts/run_autoware_map_beginner.sh`' not in (
        contributing
    )
    assert '(CODE_OF_CONDUCT.md)' in contributing
    assert '(GOVERNANCE.md)' in contributing
    assert '(SUPPORT.md)' in contributing
    assert '(SECURITY.md)' in contributing
    assert '(docs/product-contract.md)' in contributing

    issue_config = (ISSUE_TEMPLATE_DIR / 'config.yml').read_text(encoding='utf-8')
    assert 'Product Contract And Supported Scope' in issue_config
    assert 'Usage Support' in issue_config
    assert 'Report A Security Vulnerability' in issue_config

    bug_form = (ISSUE_TEMPLATE_DIR / 'bug-report.yml').read_text(
        encoding='utf-8'
    )
    assert 'lidarslam-map doctor <bag> --public-json' in bug_form
    assert 'do not substitute ordinary `--json` output' in bug_form
    assert '--preflight-only' not in bug_form

    first_map_form = (
        ISSUE_TEMPLATE_DIR / 'first-map-validation.yml'
    ).read_text(encoding='utf-8')
    for field_id in (
        'independence',
        'documentation_path',
        'release_ref',
        'environment',
        'command',
        'result',
        'verification',
        'receipt',
        'findings',
        'privacy',
    ):
        assert f'id: {field_id}' in first_map_form
    assert 'Do not upload map geometry.' in first_map_form
    assert 'drag and drop that file here' in first_map_form
    assert 'GitHub uploads it as a public' in first_map_form
    assert 'do not attach any other run artifact' in first_map_form
    receipt_block = first_map_form.split(
        '  - type: textarea\n    id: receipt\n', 1
    )[1].split('  - type:', 1)[0]
    assert 'Required for PASS reports' in receipt_block
    assert 'For FAIL reports, leave this field empty' in receipt_block
    assert 'required: false' in receipt_block


def test_first_map_form_accepts_fail_without_weakening_pass_receipts():
    """A truthful FAIL must be submitable when no receipt was produced."""
    form_path = ISSUE_TEMPLATE_DIR / 'first-map-validation.yml'
    form = yaml.safe_load(form_path.read_text(encoding='utf-8'))
    fields = {
        item['id']: item
        for item in form['body']
        if isinstance(item, dict) and 'id' in item
    }
    receipt = fields['receipt']
    privacy = fields['privacy']
    result = fields['result']
    privacy_labels = [
        option['label'] for option in privacy['attributes']['options']
    ]

    assert result['attributes']['options'] == [
        'PASS — verified first map completed',
        'FAIL — onboarding or mapping did not complete',
    ]
    assert receipt['validations']['required'] is False
    assert 'Required for PASS reports' in receipt['attributes']['description']
    assert 'For FAIL reports, leave this field empty' in (
        receipt['attributes']['description']
    )
    assert len(privacy_labels) == 3
    assert all(
        option['required'] is True
        for option in privacy['attributes']['options']
    )
    receipt_attestation = privacy_labels[1]
    assert 'either reviewed the one attached JSON receipt' in (
        receipt_attestation
    )
    assert 'or selected FAIL because no receipt was produced' in (
        receipt_attestation
    )
    assert 'attached no file' in receipt_attestation

    validation_doc = EXTERNAL_FIRST_MAP_DOC.read_text(encoding='utf-8')
    normalized_validation_doc = ' '.join(validation_doc.split())
    assert 'privacy attestation has two honest branches' in (
        normalized_validation_doc
    )
    assert 'A missing receipt never permits a PASS report' in (
        normalized_validation_doc
    )
    assert 'a FAIL report never requires inventing or attaching another' in (
        normalized_validation_doc
    )


def test_first_map_form_requires_a_redacted_command_shape():
    """Public reproducibility must not solicit private command values."""
    form_path = ISSUE_TEMPLATE_DIR / 'first-map-validation.yml'
    form = yaml.safe_load(form_path.read_text(encoding='utf-8'))
    fields = {
        item['id']: item
        for item in form['body']
        if isinstance(item, dict) and 'id' in item
    }
    command = fields['command']
    privacy = fields['privacy']
    description = command['attributes']['description']
    placeholder = command['attributes']['placeholder']

    assert command['attributes']['label'] == 'Redacted command shape'
    assert command['attributes']['render'] == 'bash'
    assert command['validations']['required'] is True
    for required_phrase in (
        'credentials',
        'private paths',
        'host or user names',
        'precise locations',
        'literal REDACTED placeholder',
        'executable, options, and non-private values',
    ):
        assert required_phrase in description
    assert 'REDACTED' in placeholder
    assert '/path/to/' not in placeholder

    redaction_attestation = privacy['attributes']['options'][0]
    assert redaction_attestation['required'] is True
    assert 'private paths' in redaction_attestation['label']
    assert 'REDACTED' in redaction_attestation['label']
    assert 'map geometry' in redaction_attestation['label']

    validation_doc = EXTERNAL_FIRST_MAP_DOC.read_text(encoding='utf-8')
    assert 'redacted command shape' in validation_doc
    assert 'literal `REDACTED` placeholder' in validation_doc
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    golden_path = GOLDEN_PATH_CLI_DOC.read_text(encoding='utf-8')
    support_doc = SUPPORT_PATH.read_text(encoding='utf-8')
    assert 'literal `REDACTED` placeholder' in getting_started
    assert 'private values replaced\nby `REDACTED`' in golden_path
    assert '**Redacted command shape**' in support_doc
    assert 'literal `REDACTED` placeholder' in support_doc


def test_benchmark_form_is_redaction_first_and_evidence_bounded():
    """Public benchmark intake must preserve metrics without private runs."""
    form_path = ISSUE_TEMPLATE_DIR / 'benchmark-report.yml'
    form = yaml.safe_load(form_path.read_text(encoding='utf-8'))
    fields = {
        item['id']: item
        for item in form['body']
        if isinstance(item, dict) and 'id' in item
    }
    dataset = fields['dataset']
    command = fields['command']
    params = fields['params']
    outputs = fields['outputs']
    privacy = fields['privacy']
    privacy_labels = [
        option['label'] for option in privacy['attributes']['options']
    ]

    assert dataset['attributes']['label'] == (
        'Public dataset or redacted input summary'
    )
    assert 'canonical dataset, sequence, source, and license' in (
        dataset['attributes']['description']
    )
    assert 'do not identify the site or upload the bag' in (
        dataset['attributes']['description']
    )
    assert command['attributes']['label'] == 'Redacted command shape'
    assert command['attributes']['render'] == 'bash'
    assert command['validations']['required'] is True
    for private_value in (
        'credentials',
        'private paths',
        'host or user names',
        'precise locations',
        'literal REDACTED placeholder',
    ):
        assert private_value in command['attributes']['description']
    assert 'REDACTED' in command['attributes']['placeholder']

    assert params['attributes']['label'] == 'Configuration summary'
    assert 'tracked or public preset' in params['attributes']['description']
    assert 'complete custom YAML' in params['attributes']['description']
    assert outputs['attributes']['label'] == (
        'Privacy-reviewed metrics evidence'
    )
    output_help = outputs['attributes']['description']
    assert 'one reviewed metrics.json or public aggregate report' in output_help
    for forbidden_artifact in (
        'local paths',
        'rosbags',
        'maps',
        'trajectories',
        'raw sensor data',
        'private-site images',
        'precise coordinates',
    ):
        assert forbidden_artifact in output_help

    assert len(privacy_labels) == 3
    assert all(
        option['required'] is True
        for option in privacy['attributes']['options']
    )
    assert 'literal REDACTED placeholder' in privacy_labels[0]
    assert 'attached no rosbag, map or trajectory geometry' in privacy_labels[1]
    assert 'or I attached no file' in privacy_labels[2]

    contributing = ' '.join(
        CONTRIBUTING_PATH.read_text(encoding='utf-8').split()
    )
    support = ' '.join(SUPPORT_PATH.read_text(encoding='utf-8').split())
    benchmarking = ' '.join(
        BENCHMARKING_DOC.read_text(encoding='utf-8').split()
    )
    for document in (contributing, support, benchmarking):
        assert 'private or custom bag' in document
        assert 'public aggregate report' in document
        assert 'complete custom parameter YAML' in document


def test_bug_form_accepts_pre_session_failure_without_fake_zip_claims():
    """Pre-session failures need diagnostics, not an impossible ZIP claim."""
    bug_form_path = ISSUE_TEMPLATE_DIR / 'bug-report.yml'
    bug_form = yaml.safe_load(bug_form_path.read_text(encoding='utf-8'))
    fields = {
        item['id']: item
        for item in bug_form['body']
        if isinstance(item, dict) and 'id' in item
    }
    preflight = fields['preflight']
    command = fields['command']
    diagnostics = fields['diagnostics']
    checks = fields['checks']
    check_labels = [
        option['label'] for option in checks['attributes']['options']
    ]

    assert command['attributes']['label'] == 'Redacted command shape'
    assert command['attributes']['render'] == 'bash'
    assert command['validations']['required'] is True
    command_help = command['attributes']['description']
    for private_value in (
        'credentials',
        'private paths',
        'host or user names',
        'precise locations',
        'literal REDACTED placeholder',
    ):
        assert private_value in command_help
    assert 'REDACTED' in command['attributes']['placeholder']

    assert preflight['attributes']['label'] == 'Public doctor evidence'
    assert preflight['attributes']['render'] == 'json'
    assert preflight['validations']['required'] is True
    preflight_help = preflight['attributes']['description']
    assert 'lidarslam-map doctor <bag> --public-json' in preflight_help
    assert 'paste the complete JSON' in preflight_help
    assert 'bag-preflight-input-error' in preflight_help
    for omitted_value in (
        'bag paths',
        'topic or frame names',
        'local commands',
        'raw data',
        'raw logs',
        'free-text messages',
    ):
        assert omitted_value in preflight_help
    assert 'do not substitute ordinary `--json` output' in preflight_help
    assert diagnostics['validations']['required'] is True
    diagnostics_help = diagnostics['attributes']['description']
    assert 'If a session exists' in diagnostics_help
    assert 'If no session was created' in diagnostics_help
    assert 'first_action_code' in diagnostics_help
    assert 'finding_codes' in diagnostics_help
    assert 'Do not paste raw doctor output, terminal history, or logs' in (
        diagnostics_help
    )
    assert len(check_labels) == 5
    assert all(
        option['required'] is True
        for option in checks['attributes']['options']
    )
    zip_attestation = check_labels[2]
    assert 'either reviewed every file in the one attached support ZIP' in (
        zip_attestation
    )
    assert 'or no session and ZIP existed' in zip_attestation
    assert 'attached no file' in zip_attestation
    public_attestation = check_labels[1]
    assert 'complete reviewed `--public-json` evidence' in public_attestation
    assert 'literal REDACTED placeholder' in public_attestation
    artifact_attestation = check_labels[3]
    assert 'no rosbag, map or trajectory geometry' in artifact_attestation
    assert 'terminal history' in artifact_attestation

    support = ' '.join(SUPPORT_PATH.read_text(encoding='utf-8').split())
    assert 'do not fabricate an empty ZIP or claim to have reviewed one' in (
        support
    )
    assert 'This pre-session path does not weaken the ZIP review requirement' in (
        support
    )

    issue_config = yaml.safe_load(
        (ISSUE_TEMPLATE_DIR / 'config.yml').read_text(encoding='utf-8')
    )
    usage_support = next(
        link
        for link in issue_config['contact_links']
        if link['name'] == 'Usage Support'
    )
    assert 'If a session exists' in usage_support['about']
    assert 'otherwise use doctor --public-json' in usage_support['about']

    contributing = ' '.join(
        CONTRIBUTING_PATH.read_text(encoding='utf-8').split()
    )
    golden_path = ' '.join(
        GOLDEN_PATH_CLI_DOC.read_text(encoding='utf-8').split()
    )
    compatibility = ' '.join(
        CLI_COMPATIBILITY_DOC.read_text(encoding='utf-8').split()
    )
    getting_started = ' '.join(
        GETTING_STARTED.read_text(encoding='utf-8').split()
    )
    for document in (support, contributing, golden_path, compatibility):
        assert 'doctor <rosbag2_dir> --public-json' in document
        assert 'public-doctor-evidence-v1' in document
    assert 'Every human bag report ends with one shell-safe' in (
        getting_started
    )
    assert 'Keep the full report local' in getting_started
    assert 'human bag report always ends with that shell-safe command' in (
        compatibility
    )
    assert 'one exact **Do this now** `lidarslam-map start` command' in (
        getting_started
    )
    assert 'withholds that start command' in getting_started
    assert 'ready report exposes one exact-input `lidarslam-map start`' in (
        compatibility
    )
    assert 'report with findings withholds that start action' in compatibility
    assert 'product report is a compact local card' in getting_started
    assert 'topic/frame names, detailed reasons' in getting_started
    assert 'default product card is bounded' in compatibility
    assert 'exact private `doctor ... --json` command' in compatibility


def test_autoware_form_never_requests_private_map_or_origin_evidence():
    """Autoware reports need useful diagnostics without location disclosure."""
    form_path = ISSUE_TEMPLATE_DIR / 'autoware-pointcloud-map.yml'
    form = yaml.safe_load(form_path.read_text(encoding='utf-8'))
    fields = {
        item['id']: item
        for item in form['body']
        if isinstance(item, dict) and 'id' in item
    }
    command = fields['command']
    verifier = fields['verifier']
    projector = fields['projector']
    artifacts = fields['artifacts']
    privacy = fields['privacy']
    privacy_labels = [
        option['label'] for option in privacy['attributes']['options']
    ]

    assert command['validations']['required'] is True
    assert 'REDACTED placeholders' in command['attributes']['description']
    assert verifier['validations']['required'] is True
    assert 'removing private paths and precise locations' in (
        verifier['attributes']['description']
    )
    assert projector['validations']['required'] is True
    projector_help = projector['attributes']['description']
    for private_origin_field in (
        'latitude',
        'longitude',
        'altitude',
        'MGRS/grid identifiers',
        'precise origin value',
    ):
        assert private_origin_field in projector_help
    assert 'REDACTED' in projector_help
    assert 'never paste the complete map_projector_info.yaml' in projector_help

    artifacts_help = artifacts['attributes']['description']
    assert 'lidarslam-map support <session_bundle>' in artifacts_help
    assert 'Do not attach a map bundle' in artifacts_help
    assert 'screenshots revealing a private place' in artifacts_help
    assert len(privacy_labels) == 3
    assert all(
        option['required'] is True
        for option in privacy['attributes']['options']
    )
    assert 'every projector-origin coordinate' in privacy_labels[0]
    assert 'pointcloud/lanelet geometry' in privacy_labels[1]
    assert 'or attached no file' in privacy_labels[2]

    for document_path in (
        CONTRIBUTING_PATH,
        SUPPORT_PATH,
        AUTOWARE_MAP_AUTHORING,
    ):
        document = ' '.join(document_path.read_text(encoding='utf-8').split())
        assert 'MGRS/grid' in document
        assert 'precise origin' in document
        assert 'screenshots revealing a private place' in document


def test_product_contract_has_bounded_official_surface():
    """The beginner product surface should stay explicit and bounded."""
    contract = PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    golden_path = GOLDEN_PATH_CLI_DOC.read_text(encoding='utf-8')
    roadmap = V09_ROADMAP_DOC.read_text(encoding='utf-8')

    assert '## Official entrypoints' in contract
    assert contract.count('| Try the fixed public demo') == 1
    assert contract.count('| Check an installation before finding a bag') == 1
    assert contract.count('| Map your own compatible rosbag2') == 1
    assert contract.count('| Return to and compare local sessions') == 1
    assert contract.count('| Prepare a maintainer or first-map report') == 1
    assert 'lidarslam-map run <rosbag2_dir> --output-dir <dir>' in contract
    assert '`lidarslam-map demo [work_dir]`' in contract
    assert 'ros2 run lidarslam lidarslam-cli' in contract
    assert '`run_first_map_demo.sh` implementation used by Docker' in contract
    assert (
        '`run_autoware_quickstart.sh` remains an advanced viewer/dogfood '
        'compatibility' in contract
    )
    assert 'Other scripts and ROS' in contract
    assert 'choice-reducing home, not another mapping workflow' in contract
    assert '`run_manifest.json`' in contract
    assert '`<output>.partial`' in golden_path
    assert 'preflight-v6.schema.json' in golden_path
    assert 'preflight-v5.schema.json' in golden_path
    assert 'preflight-v4.schema.json' in golden_path
    assert 'sensor-setup-rejection-v1.schema.json' in golden_path
    assert 'map-session-recovery-v1.schema.json' in golden_path
    assert 'map-session-index-v1.schema.json' in golden_path
    assert 'preflight-v3.schema.json' in golden_path
    assert 'preflight-v2.schema.json' in golden_path
    assert 'preflight-v1.schema.json' in golden_path
    assert 'diagnosis-v1.schema.json' in golden_path
    assert 'run-manifest-v1.schema.json' in golden_path
    assert 'run-manifest-v2.schema.json' in golden_path
    assert '--resume' in golden_path
    assert 'lidarslam-map run --help-all' in golden_path
    compatibility = CLI_COMPATIBILITY_DOC.read_text(encoding='utf-8')
    assert 'Normal help is the operator view' in compatibility
    assert 'view --help-all' in compatibility
    cli_contract = json.loads(CLI_V1_CONTRACT.read_text(encoding='utf-8'))
    assert set(cli_contract['help_modes']) == {'normal', 'all'}
    assert cli_contract['help_modes']['normal']['excludes'] == {
        'stability': ['deprecated'],
        'tiers': ['viewer-runtime'],
    }
    assert cli_contract['map_session_recovery_contract']['command'] == 'start'
    assert cli_contract['map_session_index_contract']['command'] == 'start'
    home = cli_contract['interactive_home_contract']
    assert home['routes'] == [
        'demo',
        'start',
        'sessions',
        'doctor',
        'help',
    ]
    assert 'usage exit code 2' in home['non_interactive_behavior']
    assert 'map_session_recovery.json' in contract
    assert 'session.json' in contract
    assert 'session.html' in contract
    assert 'session.html' in golden_path
    assert 'Resume never starts the SLAM workflow again.' in golden_path
    assert 'existing outputs are never overwritten' in (
        REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
    ).read_text(encoding='utf-8')
    assert '## Non-goals' in contract
    assert 'No more than three beginner-facing entrypoints' in roadmap
    assert 'Phase 1 — Golden-path UX' in roadmap


def test_generated_output_artifacts_are_local_only():
    """Generated benchmark/report artifacts should stay out of git."""
    gitignore = GITIGNORE_PATH.read_text(encoding='utf-8')
    benchmarking_doc = BENCHMARKING_DOC.read_text(encoding='utf-8')

    assert 'output/' in gitignore
    assert '/build-*/' in gitignore
    assert '/install-*/' in gitignore
    assert '/log-*/' in gitignore
    assert '/symlink_install_manifest.txt' in gitignore
    assert 'benchmark_summary.md' in benchmarking_doc
    assert 'latest_report.html' in benchmarking_doc


def test_release_metadata_and_core_package_versions_match(tmp_path: Path):
    """Release metadata should stay aligned with core package versions."""
    version = VERSION_PATH.read_text(encoding='utf-8').strip()
    changelog = CHANGELOG_PATH.read_text(encoding='utf-8')
    releasing = RELEASING_PATH.read_text(encoding='utf-8')
    release_notes = (REPO_ROOT / 'docs' / 'releases' / f'v{version}.md').read_text(
        encoding='utf-8'
    )
    release_workflow = RELEASE_WORKFLOW.read_text(encoding='utf-8')
    release_bundle_script = RELEASE_BUNDLE_SCRIPT.read_text(encoding='utf-8')
    release_promotion_script = RELEASE_PROMOTION_SCRIPT.read_text(
        encoding='utf-8'
    )
    docs_site_workflow = DOCS_SITE_WORKFLOW.read_text(encoding='utf-8')
    mkdocs_config = MKDOCS_CONFIG_PATH.read_text(encoding='utf-8')

    version_parts = version.split('.')
    assert len(version_parts) == 3
    assert all(part.isdigit() for part in version_parts)
    assert version in changelog
    assert 'VERSION="$(tr -d \'\\n\' < VERSION)"' in releasing
    assert 'git tag "v${VERSION}"' in releasing
    assert 'scripts/check_v1_readiness.py --json' in releasing
    assert 'scripts/check_v1_readiness.py --require-complete' in releasing
    pre_release = releasing.split('## Pre-Release Checklist', 1)[1].split(
        '## Automated Publication', 1
    )[0]
    for command in (
        './scripts/run_product_python_tests.sh',
        'python3 -m mkdocs build --strict',
        'lidarslam-map doctor',
        'lidarslam-map demo "${DEMO_WORK_DIR}" --viewer none',
    ):
        assert command in pre_release
    assert 'bash scripts/run_autoware_quickstart.sh' not in pre_release
    assert '`docs/autoware-map-authoring.md`' in pre_release
    assert 'Autoware-compatible' in release_notes
    candidate_banner = (
        'Release candidate status — HOLD; not published or tagged'
    )
    if candidate_banner in release_notes:
        assert '## Release decision: HOLD' in release_notes
        assert '2,474 passed / 13 skipped' in release_notes
        assert '9f8a2058a3c702f69d159079568ced8433ee3377' in (
            release_notes
        )
    else:
        assert '## Release decision: HOLD' not in release_notes
        assert 'not published or tagged' not in release_notes
        assert '## Release verification' in release_notes
    assert '2,432 passed / 13 skipped' not in release_notes
    assert 'download_ntu_viral_tnp01.sh --dry-run' in release_notes
    assert 'run_product_python_tests.sh' in release_notes
    assert 'run_release_readiness_checks.sh' in release_notes
    for profile in (
        'newer_college_math_hard',
        'ntu_viral_tnp_01',
        'mid360_gt_rtkslam_construction_seq2',
        'mid360_gt_rtkslam_construction_seq1',
        'leo_drive_applanix_velodyne_cross',
    ):
        assert profile in release_notes
    for command in (
        'lidarslam-map doctor',
        'lidarslam-map demo',
        'lidarslam-map start /path/to/rosbag2',
    ):
        assert command in release_notes
    assert 'release notes still contain the candidate-only status' in (
        release_workflow
    )
    assert 'release notes still contain a HOLD decision' in release_workflow
    assert 'release notes lack the final verification section' in (
        release_workflow
    )
    guard = release_workflow.split(
        '# RELEASE_NOTES_PUBLICATION_GUARD_BEGIN', 1
    )[1].split('# RELEASE_NOTES_PUBLICATION_GUARD_END', 1)[0]
    guard = '\n'.join(
        line[10:] if line.startswith('          ') else line
        for line in guard.splitlines()
    )

    def run_release_notes_guard(notes: str) -> subprocess.CompletedProcess:
        notes_path = tmp_path / f'release-notes-{len(notes)}.md'
        notes_path.write_text(notes, encoding='utf-8')
        return subprocess.run(
            [
                'bash',
                '-eu',
                '-o',
                'pipefail',
                '-c',
                'RELEASE_NOTES="$1"\n' + guard,
                'release-notes-guard',
                str(notes_path),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    candidate_guard = run_release_notes_guard(
        '> **Release candidate status — HOLD; not published or tagged**\n'
        '## Release decision: HOLD\n'
    )
    assert candidate_guard.returncode == 1
    assert 'candidate-only status' in candidate_guard.stderr

    missing_verification_guard = run_release_notes_guard(
        '# lidarslam_ros2 v0.9.1\n'
    )
    assert missing_verification_guard.returncode == 1
    assert 'lack the final verification section' in (
        missing_verification_guard.stderr
    )

    final_guard = run_release_notes_guard(
        '# lidarslam_ros2 v0.9.1\n\n## Release verification\n'
    )
    assert final_guard.returncode == 0
    assert final_guard.stderr == ''
    assert 'action-gh-release@v2' in release_workflow
    assert 'docker/setup-buildx-action@v4' in release_workflow
    assert 'docker/login-action@v4' in release_workflow
    assert 'docker/metadata-action@v6' in release_workflow
    assert 'docker/build-push-action@v7' in release_workflow
    assert 'actions/attest@v4' in release_workflow
    assert 'actions/download-artifact@v7' in release_workflow
    assert 'release-image-${{ matrix.ros_distro }}.json' in release_workflow
    assert 'push-by-digest=true' in release_workflow
    assert 'scripts/create_release_image_record.py' in release_workflow
    assert 'scripts/promote_release_images.py' in release_workflow
    assert 'release-promotion.json' in release_workflow
    assert 'refusing to move' in release_promotion_script
    assert 'moving_tag_mutated' in release_promotion_script
    assert 'v<VERSION>-<distro>' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
    distribution_doc = DISTRIBUTION_DOC.read_text(encoding='utf-8')
    assert 'product-build-info.json' in distribution_doc
    assert 'LIDARSLAM_SOURCE_REVISION:STRING' in distribution_doc
    assert 'LIDARSLAM_SOURCE_DIRTY:STRING' in distribution_doc
    assert 'scripts/build_release_bundle.py' in release_workflow
    for bundled_path in (
        'mkdocs.yml',
        'docs/index.md',
        'docs/assets',
        'docs/releases/',
        'docs/autoware-map-authoring.md',
        'docs/product-contract.md',
        'docs/v1-readiness.md',
        'docs/getting-started.md',
        'docs/getting-started-ja.md',
        'docs/usability-scorecard.md',
        'docs/golden-path-cli.md',
        'docs/cli-compatibility.md',
        'docs/contracts',
        'docs/operational-reliability.md',
        'docs/evidence',
        'docs/schemas',
        'docs/real-data-e2e.md',
        'docs/external-first-map-validation.md',
        'configs/real_data_e2e/driving_slam_mid360_v1.json',
        'docs/distribution.md',
        'docs/rosdistro-release.md',
        'docs/roadmap/v0.9.md',
        'docs/autoware-foxglove.md',
        'docs/social/autoware_map_authoring_post_v0.9.1.md',
        'docs/workflows.md',
        'lidarslam/images/autoware_map_loader_proof.png',
        'lidarslam/images/dynamic_object_filter_bag6_summary.svg',
        'lidarslam/images/social_autoware_map_authoring.png',
        'lidarslam/images/social_autoware_map_authoring_demo.mp4',
        'lidarslam/images/social_autoware_map_authoring_demo.en.vtt',
        'lidarslam/images/social_autoware_map_authoring_demo.manifest.json',
        'scripts/generate_social_autoware_demo_video.py',
        'scripts/generate_social_autoware_map_authoring_card.py',
    ):
        assert bundled_path in release_bundle_script
    assert 'docs/social/autoware_map_authoring_post_v0.2.2.md' not in (
        release_bundle_script
    )
    for policy in (
        'SECURITY.md',
        'SUPPORT.md',
        'CODE_OF_CONDUCT.md',
        'GOVERNANCE.md',
        'CITATION.cff',
    ):
        assert policy in release_bundle_script
    assert 'scripts/check_external_first_map_readiness.py' in (
        release_bundle_script
    )
    assert 'scripts/check_first_map_verification_package.py' in (
        release_bundle_script
    )
    assert "'docs/schemas'" in release_bundle_script
    assert 'python3 scripts/check_first_map_verification_package.py --json' in (
        release_workflow
    )
    assert 'scripts/check_v1_readiness.py' in release_bundle_script
    assert 'actions/configure-pages@v5' in docs_site_workflow
    assert 'actions/upload-pages-artifact@v4' in docs_site_workflow
    assert 'actions/deploy-pages@v4' in docs_site_workflow
    assert 'python3 -m mkdocs build --strict' in docs_site_workflow
    assert 'scripts/generate_docs_deployment_manifest.py' in (
        docs_site_workflow
    )
    assert '--source-revision "${GITHUB_SHA}"' in docs_site_workflow
    assert 'site/docs-deployment-v1.json' in docs_site_workflow
    assert (
        docs_site_workflow.count(
            "if: github.ref == 'refs/heads/develop'"
        )
        == 2
    )
    assert "- 'VERSION'" in docs_site_workflow
    assert "- 'scripts/generate_docs_deployment_manifest.py'" in (
        docs_site_workflow
    )
    assert "- 'scripts/docs_deployment_contract.py'" in docs_site_workflow
    assert "- 'scripts/check_first_map_verification_package.py'" in (
        docs_site_workflow
    )
    assert "- 'docs/schemas/first-map-verification-package-v1.schema.json'" in (
        docs_site_workflow
    )
    assert 'python3 scripts/check_first_map_verification_package.py --json' in (
        docs_site_workflow
    )
    assert 'scripts/check_public_docs_deployment.py' in release_bundle_script
    assert 'scripts/check_published_onboarding_identity.py' in (
        release_bundle_script
    )
    assert 'scripts/generate_docs_deployment_manifest.py' in (
        release_bundle_script
    )
    assert 'scripts/docs_deployment_contract.py' in release_bundle_script
    assert 'README.md' in docs_site_workflow

    rosdistro_release = ROSDISTRO_RELEASE_DOC.read_text(encoding='utf-8')
    assert f'maintained through v{version}' in rosdistro_release
    for package_name in (
        'lidarslam_msgs',
        'scanmatcher',
        'graph_based_slam',
        'lidarslam',
    ):
        assert f'| `{package_name}` | {version} |' in rosdistro_release

    package_paths = [
        REPO_ROOT / 'lidarslam' / 'package.xml',
        REPO_ROOT / 'graph_based_slam' / 'package.xml',
        REPO_ROOT / 'lidarslam_msgs' / 'package.xml',
        REPO_ROOT / 'scanmatcher' / 'package.xml',
    ]
    for path in package_paths:
        package_xml = path.read_text(encoding='utf-8')
        assert f'<version>{version}</version>' in package_xml
        changelog_rst = (path.parent / 'CHANGELOG.rst').read_text(encoding='utf-8')
        assert f'Changelog for package {path.parent.name}' in changelog_rst
        assert f'{version} (' in changelog_rst

    assert 'site_name: lidarslam_ros2 Docs' in mkdocs_config
    assert 'site_url: https://rsasaki0109.github.io/lidar_slam_ros2/' in mkdocs_config
    assert 'repo_url: https://github.com/rsasaki0109/lidar_slam_ros2' in mkdocs_config
    assert 'name: material' in mkdocs_config
    assert 'assets/stylesheets/extra.css' in mkdocs_config
    assert 'Getting Started: getting-started.md' in mkdocs_config
    assert '日本語クイックスタート: getting-started-ja.md' in mkdocs_config
    assert 'GLIM usability scorecard: usability-scorecard.md' in mkdocs_config
    assert 'Product Contract: product-contract.md' in mkdocs_config
    assert 'v1.0 Readiness: v1-readiness.md' in mkdocs_config
    assert 'Golden-path CLI: golden-path-cli.md' in mkdocs_config
    assert 'CLI compatibility: cli-compatibility.md' in mkdocs_config
    assert 'Distribution and installed CLI: distribution.md' in mkdocs_config
    assert 'Operational reliability: operational-reliability.md' in mkdocs_config
    assert (
        'Named-hardware soak evidence: evidence/real-data-soak-2026-07-28.md'
        in mkdocs_config
    )
    assert (
        'Docker first-map evidence: evidence/docker-first-map-2026-07-28.md'
        in mkdocs_config
    )
    assert (
        'Independent first-map validation: external-first-map-validation.md'
        in mkdocs_config
    )
    assert (
        'CLI install evidence: evidence/cli-v1-install-2026-07-28.md'
        in mkdocs_config
    )
    assert 'Pinned real-data E2E: real-data-e2e.md' in mkdocs_config
    assert 'Autoware-Compatible Map Authoring: autoware-map-authoring.md' in mkdocs_config
    assert 'Autoware Foxglove: autoware-foxglove.md' in mkdocs_config
    assert 'Benchmarking And Release Gate: benchmarking.md' in mkdocs_config
    assert f'v{version}: releases/v{version}.md' in mkdocs_config
    assert 'v0.2.2: releases/v0.2.2.md' in mkdocs_config
    assert (
        'v0.9.1 Candidate Media Kit: '
        'social/autoware_map_authoring_post_v0.9.1.md'
        in mkdocs_config
    )
    assert 'v0.2.2 Post Kit:' not in mkdocs_config
    assert 'rosdistro Binary Release: rosdistro-release.md' in mkdocs_config
    assert 'v0.9 Product Foundation: roadmap/v0.9.md' in mkdocs_config

    citation = CITATION_PATH.read_text(encoding='utf-8')
    assert f'version: {version}' in citation
    assert 'license: BSD-2-Clause' in citation


def test_docker_workflow_separates_verification_from_tag_publication():
    """Manual and PR builds must not receive package-write authority."""
    workflow = DOCKER_WORKFLOW.read_text(encoding='utf-8')
    workflow_permissions, jobs = workflow.split('\njobs:\n', 1)
    verify, publish = jobs.split('\n  publish:\n', 1)

    assert 'workflow_dispatch:' in workflow_permissions
    assert 'branches:\n      - develop' in workflow_permissions
    assert 'contents: read' in workflow_permissions
    assert 'packages: write' not in workflow_permissions
    assert 'attestations: write' not in workflow_permissions
    assert 'id-token: write' not in workflow_permissions

    assert "if: github.event_name != 'push'" in verify
    assert 'name: build (${{ matrix.ros_distro }})' in verify
    assert 'permissions:\n      contents: read' in verify
    assert 'push: false' in verify
    assert 'load: true' in verify
    assert 'sbom: false' in verify
    assert 'provenance: false' in verify
    assert 'docker/login-action' not in verify
    assert 'actions/attest' not in verify
    assert 'packages: write' not in verify
    assert 'Published: \\`no\\`' in verify

    assert "if: github.event_name == 'push'" in publish
    assert 'name: build and push (${{ matrix.ros_distro }})' in publish
    assert 'packages: write' in publish
    assert 'attestations: write' in publish
    assert 'id-token: write' in publish
    assert 'push: true' in publish
    assert 'sbom: true' in publish
    assert 'provenance: mode=max' in publish
    assert 'docker/login-action@v4' in publish
    assert 'actions/attest@v4' in publish


def test_candidate_image_workflow_is_default_branch_digest_only():
    """E2 candidates need a trusted gate without any tag authority."""
    workflow = CANDIDATE_IMAGE_WORKFLOW.read_text(encoding='utf-8')
    workflow_permissions, jobs = workflow.split('\njobs:\n', 1)
    contract, remainder = jobs.split('\n  authorize:\n', 1)
    authorize, remainder = remainder.split('\n  publish:\n', 1)
    publish, verify_set = remainder.split('\n  verify-set:\n', 1)
    distribution = DISTRIBUTION_DOC.read_text(encoding='utf-8')

    assert 'repository_dispatch:' in workflow_permissions
    assert 'e2-publish-candidate-image' in workflow_permissions
    assert 'workflow_dispatch:' not in workflow_permissions
    assert 'permissions:\n  contents: read' in workflow_permissions
    assert 'packages: write' not in workflow_permissions
    assert 'packages: write' not in contract
    assert 'packages: write' not in authorize
    assert 'packages: write' not in verify_set
    assert publish.count('packages: write') == 1
    assert 'pull-requests: read' in authorize
    assert 'refs/heads/develop' in authorize
    assert 'E2_IMMUTABLE_DIGEST_ONLY' in authorize
    assert 'check-runs?filter=latest&per_page=100' in authorize
    assert 'push-by-digest=true' in publish
    assert 'name-canonical=true' in publish
    assert 'docker buildx imagetools create' not in workflow
    assert 'Tags created: \\`none\\`' in publish
    assert 'registry_retention_status' in (
        (REPO_ROOT / 'scripts' / 'create_candidate_image_record.py')
        .read_text(encoding='utf-8')
    )
    assert 'repository_dispatch' in distribution
    assert 'E2_IMMUTABLE_DIGEST_ONLY' in distribution
    assert 'prepare_candidate_trial.py' in distribution
    assert 'candidate-trial-preparation-v1.schema.json' in distribution
    assert 'run_candidate_trial.py' in distribution
    assert 'candidate-trial-execution-v1.schema.json' in distribution
    assert 'audit_candidate_image_set.py' in distribution
    assert 'REMOTE_AUDIT_PASS' in distribution
    assert '--candidate-evidence-dir' in distribution
    assert 'byte-compares all four SHA-256' in distribution
    assert 'temporary directory' in distribution


def test_docs_cover_autoware_and_release_gate_keywords():
    """The adoption docs should mention the supported operator workflows."""
    autoware_doc = AUTOWARE_QUICKSTART.read_text(encoding='utf-8')
    getting_started_doc = GETTING_STARTED.read_text(encoding='utf-8')
    autoware_map_doc = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    autoware_foxglove_doc = AUTOWARE_FOXGLOVE.read_text(encoding='utf-8')
    benchmarking_doc = BENCHMARKING_DOC.read_text(encoding='utf-8')
    comparison_doc = COMPARISON_DOC.read_text(encoding='utf-8')
    distribution_doc = DISTRIBUTION_DOC.read_text(encoding='utf-8')
    reliability_doc = OPERATIONAL_RELIABILITY_DOC.read_text(encoding='utf-8')
    timestamp_order_evidence = TIMESTAMP_ORDER_EVIDENCE_DOC.read_text(
        encoding='utf-8'
    )
    real_data_e2e_doc = REAL_DATA_E2E_DOC.read_text(encoding='utf-8')

    assert 'lidarslam-map doctor' in getting_started_doc
    assert 'lidarslam-map demo' in getting_started_doc
    assert 'no arguments on an interactive terminal' in getting_started_doc
    assert 'lidarslam-map run' in getting_started_doc
    assert 'lidarslam-map inspect' in getting_started_doc
    assert 'bash scripts/run_first_map_demo.sh' in getting_started_doc
    assert 'rko_lio_graph_mid360_preset' in getting_started_doc
    assert 'first_map_validation_receipt.json' in getting_started_doc
    assert (
        'lidarslam-map report /path/to/session_bundle'
        in getting_started_doc
    )
    assert 'LIDARSLAM_HOST_UID' in getting_started_doc
    assert 'LIDARSLAM_HOST_GID' in getting_started_doc
    assert 'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble' in getting_started_doc
    assert 'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy' in getting_started_doc
    assert 'v0.9.1` release candidate is not published or tagged yet' in getting_started_doc
    assert 'periodic' in getting_started_doc
    assert 'Bind-mounted output ownership' in distribution_doc
    assert 'Creative Commons Attribution 4.0' in real_data_e2e_doc
    first_map_evidence = DOCKER_FIRST_MAP_EVIDENCE_DOC.read_text(encoding='utf-8')
    assert 'FINAL_' not in first_map_evidence
    assert 'PR #406' in first_map_evidence
    assert 'PR #407' in first_map_evidence
    assert 'independent-user validations' in first_map_evidence
    first_map_program = EXTERNAL_FIRST_MAP_DOC.read_text(encoding='utf-8')
    first_map_ledger = json.loads(
        EXTERNAL_FIRST_MAP_LEDGER.read_text(encoding='utf-8')
    )
    first_map_schema = json.loads(
        EXTERNAL_FIRST_MAP_SCHEMA.read_text(encoding='utf-8')
    )
    assert 'Independent First-map Validation issue form' in first_map_program
    assert 'lidarslam-map report /path/to/session_bundle' in (
        first_map_program
    )
    assert 'no write or network request' in first_map_program
    assert 'not edit old session evidence merely' in first_map_program
    assert '--require-complete' in first_map_program
    assert 'first_map_validation_receipt.md' in first_map_program
    assert 'create_first_map_validation_receipt.py' in first_map_program
    assert 'first-map-validation-receipt-v1' in first_map_program
    assert 'Do not publish map geometry' in first_map_program
    assert 'Privacy-bounded JSON receipt' in first_map_program
    assert 'GitHub stores issue' in first_map_program
    assert 'attachments publicly' in first_map_program
    assert 'Download `first_map_validation_receipt.json` from the public issue' in (
        first_map_program
    )
    assert first_map_ledger['schema_version'] == 1
    assert first_map_ledger['required_validations'] == 3
    assert isinstance(first_map_ledger['validations'], list)
    assert first_map_schema['properties']['required_validations']['const'] == 3
    assert first_map_schema['definitions']['validation']['properties'][
        'independent_attestation'
    ]['const'] is True
    assert "'docs/evidence'," in RELEASE_BUNDLE_SCRIPT.read_text(
        encoding='utf-8'
    )
    assert (
        'git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git'
        in getting_started_doc
    )
    assert 'run_autoware_quickstart.sh' in autoware_doc
    assert 'Getting Started' in autoware_doc
    assert 'preflight_autoware_map_bag.py' in autoware_doc
    assert 'run_autoware_map_beginner.sh' in autoware_doc
    assert 'run_autoware_map_from_bag.py' in autoware_doc
    assert 'diagnose_autoware_map_run.py' in autoware_doc
    assert 'Autoware-Compatible Map Authoring' in autoware_doc
    assert 'download_ntu_viral_tnp01.sh' in autoware_doc
    assert 'download_ntu_viral_tnp01.sh --dry-run' in autoware_doc
    assert 'run_rko_lio_graph_autoware_dogfood.sh' in autoware_doc
    assert 'run_graph_slam_pointcloud_map_in_autoware.sh' in autoware_doc
    assert 'projector_type: Local' in autoware_doc
    assert 'Autoware Foxglove' in autoware_doc
    assert 'pointcloud_map/' in autoware_map_doc
    assert 'map_projector_info.yaml' in autoware_map_doc
    assert 'Choose One First Step' in autoware_map_doc
    assert 'lidarslam-map doctor /path/to/rosbag2' in autoware_map_doc
    assert 'lidarslam-map start /path/to/rosbag2' in autoware_map_doc
    assert 'lidarslam-map sessions' in autoware_map_doc
    assert 'lidarslam-map support /path/to/session_bundle' in autoware_map_doc
    assert 'Current Publication Boundary' in autoware_map_doc
    assert 'not published or tagged yet' in autoware_map_doc
    assert 'foxglove_bridge' in autoware_foxglove_doc
    assert 'prepare_foxglove_bridge_prefix.sh' in autoware_foxglove_doc
    assert 'run_autoware_pointcloud_map_foxglove.sh' in autoware_foxglove_doc
    assert 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in autoware_foxglove_doc

    workflows_doc = WORKFLOWS_DOC.read_text(encoding='utf-8')
    assert 'download_ntu_viral_tnp01.sh --dry-run' in workflows_doc
    assert 'Required Input Topics' in workflows_doc
    assert 'sensor_msgs/msg/PointCloud2' in workflows_doc
    assert 'sensor_msgs/msg/Imu' in workflows_doc
    assert 'lidarslam_msgs/msg/MapArray' in workflows_doc
    assert 'wheel odometry / vehicle speed topic fusion' in workflows_doc
    assert 'gnss_topic' in workflows_doc
    assert 'gnss_use_covariance_weighting' in workflows_doc
    assert 'gnss_header_stamp_max_skew_sec' in workflows_doc
    assert 'RTK-like' in workflows_doc
    assert 'inspect_navsatfix_covariance.py' in workflows_doc
    assert 'inspect_applanix_gsof50_quality.py' in workflows_doc
    assert 'convert_applanix_gsof_to_navsatfix_bag.py' in workflows_doc
    assert 'convert_applanix_gsof_to_imu_bag.py' in workflows_doc
    assert 'extract_applanix_gsof49_reference.py' in workflows_doc
    assert 'extract_static_transform_from_bag.py' in workflows_doc
    assert 'prepare_velodyne_pointcloud_overlay.sh' in workflows_doc
    assert 'run_open_data_gnss_smoke.sh' in workflows_doc
    assert 'run_open_data_applanix_velodyne_gnss_smoke.sh' in workflows_doc
    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in workflows_doc
    assert 'run_open_data_classic_path_benchmark_suite.sh' in workflows_doc
    assert 'run_open_data_packet_imu_deskew_validation_matrix.sh' in workflows_doc
    assert 'run_dynamic_object_filter_benchmark.sh' in workflows_doc
    assert 'velodyne_msgs/msg/VelodyneScan' in workflows_doc
    assert 'Odometry and TF: two separate contracts' in workflows_doc
    assert 'timeout 5s ros2 topic echo --once' in workflows_doc
    assert 'ros2 topic echo --once --timeout' not in workflows_doc
    assert 'ros2 topic echo --once --timeout' not in getting_started_doc

    assert 'download_ntu_viral_tnp01.sh' in benchmarking_doc
    assert 'download_ntu_viral_tnp01.sh --dry-run' in benchmarking_doc
    assert 'run_rko_lio_graph_benchmark.sh' in benchmarking_doc
    assert 'run_radarless_tunnel_ab.sh' in benchmarking_doc
    assert 'evaluate_degeneracy_trajectory.py' in benchmarking_doc
    assert 'run_rko_lio_mid360_crossval_benchmark.sh' in benchmarking_doc
    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in benchmarking_doc
    assert 'run_open_data_classic_path_benchmark_suite.sh' in benchmarking_doc
    assert 'run_open_data_packet_imu_deskew_validation_matrix.sh' in benchmarking_doc
    assert 'run_dynamic_object_filter_benchmark.sh' in benchmarking_doc
    assert 'generate_exploration_closeout_report.py' in benchmarking_doc
    assert 'all-sensors-bag6' in benchmarking_doc
    assert 'classic_path_report.md' in benchmarking_doc
    assert 'exploration_closeout_report_20260327.md' in benchmarking_doc
    assert 'generate_classic_path_report.py' in benchmarking_doc
    assert 'run_place_recognition_benchmark.sh' in benchmarking_doc
    assert 'generate_place_recognition_report.py' in benchmarking_doc
    assert 'generate_packet_imu_deskew_validation_report.py' in benchmarking_doc
    assert 'generate_dynamic_object_filter_report.py' in benchmarking_doc
    assert 'run_release_readiness_checks.sh' in benchmarking_doc
    assert 'docs/comparison.md' in benchmarking_doc
    assert 'generate_v2_beta_readiness_report.py' in benchmarking_doc
    assert 'generate_map_authoring_report.py' in benchmarking_doc
    assert 'create_map_authoring_submission_bundle.sh' in benchmarking_doc
    assert 'map_qa_summary.md' in benchmarking_doc
    assert 'generate_stress_validation_report.py' in benchmarking_doc
    assert 'write_aligned_trajectory_metrics.py' in benchmarking_doc
    assert '--write-svg' in benchmarking_doc
    assert '--profile failing' in benchmarking_doc
    assert 'Capability Comparison' in comparison_doc
    assert 'Current Default Position' in comparison_doc
    assert 'ros2 run lidarslam lidarslam' in distribution_doc
    assert 'ros2 run lidarslam lidarslam-cli' in distribution_doc
    assert 'Humble amd64' in distribution_doc
    assert 'Jazzy amd64' in distribution_doc
    assert ':v<VERSION>-<distro>' in distribution_doc
    assert 'release-image-humble.json' in distribution_doc
    assert 'release-image-jazzy.json' in distribution_doc
    assert 'gh attestation verify' in distribution_doc
    assert 'BuildKit provenance' in distribution_doc
    assert 'There is currently no supported' in distribution_doc
    assert 'rko_lio' in distribution_doc
    assert 'Official PRBonn `rko_lio 0.3.2-1` passed' in (
        distribution_doc
    )
    rosdistro_release_doc = ROSDISTRO_RELEASE_DOC.read_text(encoding='utf-8')
    assert '`rko_lio` | PRBonn `0.3.2-1` is registered' in (
        rosdistro_release_doc
    )
    assert 'main has Humble `0.3.2` and Jazzy `0.2.0`' in (
        rosdistro_release_doc
    )
    assert '`PRBonn/rko_lio`' in rosdistro_release_doc
    assert '`ndt_omp_ros2`' in rosdistro_release_doc
    assert '`REVIEW_REQUIRED`' in rosdistro_release_doc
    assert '`include/pclomp/*`' in rosdistro_release_doc
    assert '`lib/libndt_omp.so`' in rosdistro_release_doc
    ndt_review_evidence = (
        REPO_ROOT
        / 'docs'
        / 'evidence'
        / 'ndt-omp-release-review-2026-08-12.md'
    ).read_text(encoding='utf-8')
    assert 'upstream convergence' in ndt_review_evidence
    assert 'not safely co-installable' in ndt_review_evidence
    assert '5495fd9214945afcb4b35d5a1da385e405c52bf9' in (
        ndt_review_evidence
    )
    assert '109/109 passing' in ndt_review_evidence
    assert 'c090b8f2228b21dcf30650114f9638f38497ca5a0214e3e6063a53aa7bef66b1' in (
        ndt_review_evidence
    )
    assert 'corrected five-file patch replaces all eleven direct' in (
        ndt_review_evidence
    )
    assert 'graph_based_slam' in ndt_review_evidence
    assert 'check_canonical_ndt_convergence.py' in rosdistro_release_doc
    assert 'Prepared reviewer response' in ndt_review_evidence
    lidarslam_package = (
        REPO_ROOT / 'lidarslam' / 'package.xml'
    ).read_text(encoding='utf-8')
    assert '<exec_depend version_gte="0.3.2">rko_lio</exec_depend>' in (
        lidarslam_package
    )
    official_rko_evidence = json.loads(
        OFFICIAL_RKO_EVIDENCE_JSON.read_text(encoding='utf-8')
    )
    assert official_rko_evidence['status'] == 'passed'
    assert official_rko_evidence['workflow']['run_id'] == 30412938777
    assert official_rko_evidence['contract']['source_rko_lio_present'] is False
    assert {
        candidate['ros_distro']
        for candidate in official_rko_evidence['candidates']
    } == {'humble', 'jazzy'}
    assert all(
        candidate['deb_version'].startswith('0.3.2-1')
        and candidate['e2e']['status'] == 'PASS'
        and candidate['e2e']['checks_passed'] == 18
        and candidate['e2e']['checks_failed'] == 0
        for candidate in official_rko_evidence['candidates']
    )
    apt_readiness = json.loads(
        ROS_APT_READINESS_20260812.read_text(encoding='utf-8')
    )
    assert apt_readiness['status'] == 'IN_PROGRESS'
    assert apt_readiness['distros']['humble']['main']['rko-lio']['ready'] is True
    assert apt_readiness['distros']['jazzy']['main']['rko-lio']['ready'] is False
    assert apt_readiness['distros']['jazzy']['testing']['rko-lio']['ready'] is True
    assert all(
        apt_readiness['distros'][distro][channel]['ndt-omp-ros2']['ready']
        is False
        for distro in ('humble', 'jazzy')
        for channel in ('main', 'testing')
    )
    assert 'SIGTERM' in reliability_doc
    assert 'exit `143`' in reliability_doc
    assert 'Automated failure injection' in reliability_doc
    assert 'timestamp reversal' in reliability_doc
    assert '100,000-record per-topic bound' in timestamp_order_evidence
    assert '750,000,000 ns' in timestamp_order_evidence
    assert "'docs/evidence'," in RELEASE_BUNDLE_SCRIPT.read_text(
        encoding='utf-8'
    )
    assert 'disk-pressure' in reliability_doc
    assert '--min-free-space-gib' in reliability_doc
    assert '5 GiB' in reliability_doc
    assert 'No space left on device' in reliability_doc
    assert 'raw_fallocate' in reliability_doc
    assert '32 MiB Docker tmpfs' in reliability_doc
    assert 'run_bounded_filesystem_exhaustion.py' in reliability_doc
    assert 'bounded-filesystem-exhaustion-v1.schema.json' in reliability_doc
    assert 'scripts/run_map_soak.py' in reliability_doc
    assert '--soak-profile one-hour' in reliability_doc
    assert '--hardware-label' in reliability_doc
    assert '--max-peak-rss-mib' in reliability_doc
    assert '--max-iteration-secs' in reliability_doc
    assert '--telemetry-interval-secs' in reliability_doc
    assert 'soak-report-v4.schema.json' in reliability_doc
    assert 'soak-report-v3.schema.json' in reliability_doc
    assert 'iteration_duration_within_budget' in reliability_doc
    assert 'provenance_recorded' in reliability_doc
    assert 'Schemas v1, v2 and v3 remain published' in reliability_doc
    assert 'SIGKILL' in reliability_doc
    assert 'GNU `time`' in reliability_doc
    assert '3,600 or 28,800 seconds' in reliability_doc
    assert '(evidence/real-data-soak-2026-07-28.md)' in reliability_doc
    soak_evidence = SOAK_EVIDENCE_DOC.read_text(encoding='utf-8')
    soak_ledger = json.loads(SOAK_EVIDENCE_JSON.read_text(encoding='utf-8'))
    assert '671 / 671' in soak_evidence
    assert '28,834.115 s' in soak_evidence
    assert soak_ledger['evidence_version'] == 1
    assert soak_ledger['software']['git_commit'] == (
        '0ec55575ffc16eb008e9f24bd6c6f24700bf2f8a'
    )
    assert [run['profile'] for run in soak_ledger['runs']] == [
        'one-hour',
        'eight-hour',
    ]
    assert all(run['status'] == 'passed' for run in soak_ledger['runs'])
    assert all(all(run['checks'].values()) for run in soak_ledger['runs'])
    assert '(operational-reliability.md)' in (
        PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    )
    assert 'driving_slam_mid360_v1' in real_data_e2e_doc
    assert '517088133' in real_data_e2e_doc
    assert '0836c50859bb1af591966b69da166186' in real_data_e2e_doc
    assert 'validate_real_data_e2e.py' in real_data_e2e_doc
    assert '(real-data-e2e.md)' in (
        PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    )


def test_canonical_map_authoring_page_has_one_beginner_contract():
    """The shortest map page must not revive lower-level beginner forks."""
    map_doc = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    docs_index = DOCS_INDEX_PATH.read_text(encoding='utf-8')

    official_commands = (
        'lidarslam-map doctor',
        'lidarslam-map demo',
        'lidarslam-map start /path/to/rosbag2',
        'lidarslam-map run /path/to/rosbag2 --output-dir',
        'lidarslam-map sessions',
        'lidarslam-map compare',
        'lidarslam-map support /path/to/session_bundle',
    )
    for command in official_commands:
        assert command in map_doc

    retired_beginner_entrypoints = (
        'preflight_autoware_map_bag.py',
        'run_autoware_map_beginner.sh',
        'run_autoware_map_from_bag.py',
        'run_autoware_quickstart.sh',
        'verify_autoware_map.py',
        'diagnose_autoware_map_run.py',
    )
    for entrypoint in retired_beginner_entrypoints:
        assert entrypoint not in map_doc

    assert 'assets/images/autoware_map_loader_proof.png' in map_doc
    assert 'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble' in map_doc
    assert 'not published or tagged yet' in map_doc
    assert 'Lower-level launch files and repository helpers are advanced' in (
        map_doc
    )
    assert (
        'href="autoware-map-authoring.html">Map Your Bag</a>'
        in docs_index
    )
    assert '<h3>Advanced Autoware Compatibility</h3>' in docs_index
    assert 'href="autoware-quickstart.html">Run The Quickstart</a>' not in (
        docs_index
    )


def test_real_data_e2e_workflow_is_pinned_bounded_and_non_geometry():
    """The scheduled public-bag workflow must be pinned and bounded."""
    workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'real-data-e2e.yml'
    ).read_text(encoding='utf-8')

    assert "cron: '17 17 * * *'" in workflow
    assert 'workflow_dispatch:' in workflow
    assert 'container: ros:jazzy-ros-core' in workflow
    assert 'timeout-minutes: 45' in workflow
    assert 'actions/cache@v5' in workflow
    assert '0836c50859bb1af591966b69da166186' in workflow
    assert 'timeout --signal=TERM --kill-after=30s 20m' in workflow
    assert 'lidarslam-map doctor' in workflow
    assert 'lidarslam-map run' in workflow
    assert 'validate_real_data_e2e.py' in workflow
    assert 'output/real-data-e2e/run/map.pcd' not in workflow
    assert 'output/real-data-e2e/run/traj_raw.tum' not in workflow


def test_default_container_workflow_uses_exact_trusted_checkout():
    """Container jobs test the exact PR head from a trusted checkout."""
    workflow = MAIN_CI_WORKFLOW.read_text(encoding='utf-8')
    default_workflow = workflow.split('  default-workflow:', 1)[1].split(
        '  release-readiness:', 1
    )[0]

    python_dependencies = default_workflow.index(
        '- name: Install Python test dependencies'
    )
    checkout = default_workflow.index('uses: actions/checkout@v6')
    safe_directory = default_workflow.index(
        'git config --global --add safe.directory "${GITHUB_WORKSPACE}"'
    )
    rosdep = default_workflow.index('- name: Initialize rosdep')
    assert 'python3-pip' in default_workflow
    assert 'iproute2' in default_workflow
    assert 'rosbags==0.11.0' in default_workflow
    assert (
        "repository: ${{ github.event_name == 'pull_request' && "
        'github.event.pull_request.head.repo.full_name || github.repository }}'
        in default_workflow
    )
    assert (
        "ref: ${{ github.event_name == 'pull_request' && "
        'github.event.pull_request.head.sha || github.sha }}'
        in default_workflow
    )
    assert 'fetch-depth: 0' in default_workflow
    assert 'for attempt in 1 2 3' in default_workflow
    assert 'rosdep update failed after ${attempt} attempts' in default_workflow
    assert 'if [[ -d build ]]; then' in default_workflow
    assert 'No build directory; test results are unavailable' in default_workflow
    assert python_dependencies < checkout < safe_directory < rosdep


def test_docs_metadata_workflow_uses_exact_head_and_fetches_release_tags():
    """Publication audits need the PR head, not its synthetic merge commit."""
    workflow = MAIN_CI_WORKFLOW.read_text(encoding='utf-8')
    docs_job = workflow.split('  docs-and-release-metadata:', 1)[1].split(
        '  default-workflow:', 1)[0]

    assert 'uses: actions/checkout@v6' in docs_job
    assert (
        "repository: ${{ github.event_name == 'pull_request' && "
        'github.event.pull_request.head.repo.full_name || github.repository }}'
        in docs_job
    )
    assert (
        "ref: ${{ github.event_name == 'pull_request' && "
        'github.event.pull_request.head.sha || github.sha }}'
        in docs_job
    )
    assert 'fetch-depth: 0' in docs_job


def test_release_readiness_checkout_is_exact_head_and_tag_aware():
    """Bundle rehearsal must validate the candidate, not a PR merge ref."""
    workflow = MAIN_CI_WORKFLOW.read_text(encoding='utf-8')
    readiness_job = workflow.split('  release-readiness:', 1)[1].split(
        '  release-readiness-threshold-guard:', 1
    )[0]
    exact_ref = (
        "github.event_name == 'pull_request' && "
        'github.event.pull_request.head.sha || github.sha'
    )

    assert 'uses: actions/checkout@v6' in readiness_job
    assert (
        "repository: ${{ github.event_name == 'pull_request' && "
        'github.event.pull_request.head.repo.full_name || github.repository }}'
        in readiness_job
    )
    assert f'ref: ${{{{ {exact_ref} }}}}' in readiness_job
    assert 'fetch-depth: 0' in readiness_job
    assert 'fetch-tags: true' in readiness_job
    assert 'https://github.com/${GITHUB_REPOSITORY}.git' in readiness_job
    assert 'test "$(git rev-parse HEAD)" = "${EXPECTED_SHA}"' in readiness_job
    assert "git rev-parse 'refs/tags/v0.9.0^{commit}'" in readiness_job
    assert '0df0c4a86df9f68a894c83f8342e4107c3d23b0f' in readiness_job


def test_official_rko_binary_gate_is_release_shaped_and_version_pinned():
    """The adoption gate must not accidentally build the source submodule."""
    workflow = OFFICIAL_RKO_COMPATIBILITY_WORKFLOW.read_text(
        encoding='utf-8'
    )
    release_doc = ROSDISTRO_RELEASE_DOC.read_text(encoding='utf-8')

    assert 'RKO_RELEASE_VERSION: 0.3.2-1' in workflow
    assert 'ros2-testing-apt-source' in workflow
    assert '/etc/apt/sources.list.d/ros2-testing.list' not in workflow
    assert '- develop' in workflow
    assert 'lidarslam/param/rko_lio_mid360.yaml' in workflow
    assert 'container: ros:humble-ros-core' in workflow
    assert 'container: ros:jazzy-ros-core' in workflow
    assert 'submodules: false' in workflow
    assert 'safe.directory "${GITHUB_WORKSPACE}"' in workflow
    assert 'git submodule update --init Thirdparty/ndt_omp_ros2' in workflow
    assert 'test ! -e Thirdparty/rko_lio/package.xml' in workflow
    assert 'git submodule update --init Thirdparty/rko_lio' not in workflow
    assert '-DBUILD_TESTING=OFF' in workflow
    assert 'test "${prefix}" = "/opt/ros/${{ matrix.ros_distro }}"' in workflow
    assert 'executable_sha256' in workflow
    assert 'timeout --signal=TERM --kill-after=30s 20m' in workflow
    assert 'validate_real_data_e2e.py' in workflow
    assert 'output/official-rko-compatibility/run/map.pcd' not in workflow
    assert 'output/official-rko-compatibility/run/traj_raw.tum' not in workflow
    assert 'official-rko-binary-compatibility.yml' in release_doc
    assert 'MID-360 preset' in release_doc
    assert 'NTU-VIRAL preset is also outside' in release_doc


def test_container_distribution_builds_and_attests_both_supported_distros():
    """Container CI and releases must prove both supported binary paths."""
    docker_workflow = DOCKER_WORKFLOW.read_text(encoding='utf-8')
    release_workflow = RELEASE_WORKFLOW.read_text(encoding='utf-8')
    dockerfile = (REPO_ROOT / 'Dockerfile').read_text(encoding='utf-8')

    for workflow in (docker_workflow, release_workflow):
        assert '- humble' in workflow
        assert '- jazzy' in workflow
        assert 'linux/amd64' in workflow or workflow == docker_workflow
        assert 'docker/build-push-action@v7' in workflow
        assert 'actions/attest@v4' in workflow
        assert 'attestations: write' in workflow
        assert 'artifact-metadata: write' in workflow
        assert 'id-token: write' in workflow
        assert 'sbom:' in workflow
        assert 'provenance:' in workflow
        assert 'lidarslam-map --version' in workflow
        assert 'report <output>' in workflow
        assert 'lidarslam-map start --help' in workflow
        assert "grep -Fq -- '--map-output-dir'" in workflow
        assert 'LIDARSLAM_SOURCE_REVISION=${{ steps.source.outputs.revision }}' in workflow
        assert 'LIDARSLAM_SOURCE_DIRTY=false' in workflow
        assert 'product-build-info.json' in workflow
        assert 'OBSERVED_REVISION' in workflow

    assert 'ARG LIDARSLAM_SOURCE_REVISION=' in dockerfile
    assert 'ARG LIDARSLAM_SOURCE_DIRTY=' in dockerfile
    assert '-DLIDARSLAM_SOURCE_REVISION:STRING=' in dockerfile
    assert '-DLIDARSLAM_SOURCE_DIRTY:STRING=' in dockerfile

    assert 'load: true' in docker_workflow
    assert '.github/workflows/release.yml' in docker_workflow
    assert 'needs:' in release_workflow
    assert '- images' in release_workflow
    assert 'v<VERSION>-<distro>' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
    assert '${{ needs.metadata.outputs.tag_name }}-${{ matrix.ros_distro }}' in (
        release_workflow
    )
    assert 'subject-digest: ${{ steps.image.outputs.digest }}' in release_workflow
    assert 'docker buildx imagetools inspect' in release_workflow
    assert 'release_image_evidence/*.json' in release_workflow
    assert 'scripts/plan_image_rollback.py' in release_workflow
    assert 'rollback-plan-${{ matrix.ros_distro }}.json' in release_workflow
    assert 'lidarslam-map rollback-plan' in (
        DISTRIBUTION_DOC.read_text(encoding='utf-8')
    )
    assert 'scripts/docker_map_bag.sh' in docker_workflow
    assert "'scripts/docker_map_bag.sh'" in RELEASE_BUNDLE_SCRIPT.read_text(
        encoding='utf-8'
    )
    assert 'scripts/build_docker_launcher_asset.py' in release_workflow
    assert "'scripts/build_docker_launcher_asset.py'" in (
        RELEASE_BUNDLE_SCRIPT.read_text(encoding='utf-8')
    )
    assert '--output lidarslam-map-docker' in release_workflow
    assert 'lidarslam-map-docker --version' in release_workflow
    assert 'subject-path: |' in release_workflow


def test_container_own_bag_route_is_read_only_and_non_root():
    """Container users need one safe own-bag route without a source build."""
    readme = README_PATH.read_text(encoding='utf-8')
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    distribution = DISTRIBUTION_DOC.read_text(encoding='utf-8')

    assert '[Docker Own-Bag Map](docs/getting-started.md#docker-own-bag-map)' in readme
    assert DOCKER_MAP_BAG_SCRIPT.is_file()
    assert 'bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2' in readme

    for document in (getting_started, distribution):
        assert 'bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2' in document
        assert '--dry-run' in document
        assert 'read-only' in document
        assert 'external container networking' in document
        assert 'UID/GID' in document
        assert 'lidarslam-map start' in document
        assert 'lidarslam-map run /input --guided' not in document

    assert 'Clone-free Docker launcher release gate' in getting_started
    assert 'gh attestation verify ./lidarslam-map-docker' in getting_started
    assert 'Standalone launcher asset for the next release' in distribution
    assert 'Releases from v0.9.1 require the seventh launcher asset' in (
        distribution
    )

    launcher = DOCKER_MAP_BAG_SCRIPT.read_text(encoding='utf-8')
    assert '--user "$(id -u):$(id -g)"' in launcher
    assert 'docker run --rm --pull=never --network none' in launcher
    assert 'dst=/input,readonly' in launcher
    assert 'ROS_LOG_DIR=/output/ros-logs' in launcher
    assert 'lidarslam-map start /input' in launcher
    assert '--output-dir /output/setup' in launcher
    assert '--map-output-dir /output/map' in launcher
    assert '/var/run/docker.sock' not in launcher
    assert 'LIDARSLAM_DOCKER_LAUNCHER_VERSION="development"' in launcher
    assert 'LIDARSLAM_DOCKER_LAUNCHER_REVISION="working-tree"' in launcher
    assert '--version' in launcher
    assert 'LIDARSLAM_DOCKER_LAUNCHER_VERSION}-${ROS_DISTRO}' in launcher

    assert '[imu-input-missing]' in getting_started
    assert 'concrete `Next:` action' in getting_started


def test_source_quickstart_bootstraps_dependencies_and_keeps_dev_tests():
    """Beginner builds should be fast while contributor checks stay complete."""
    readme = README_PATH.read_text(encoding='utf-8')
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    distribution = DISTRIBUTION_DOC.read_text(encoding='utf-8')
    onboarding = ONBOARDING_TRIAL_EXECUTION_DOC.read_text(encoding='utf-8')
    onboarding_trials = ONBOARDING_TRIALS_DOC.read_text(encoding='utf-8')
    workflows = WORKFLOWS_DOC.read_text(encoding='utf-8')
    helper = 'bash src/lidar_slam_ros2/scripts/install_source_dependencies.sh'
    quickstart = 'source_quickstart.sh'
    fast_build = '-DBUILD_TESTING=OFF'

    assert SOURCE_DEPENDENCIES_SCRIPT.is_file()
    assert SOURCE_QUICKSTART_SCRIPT.is_file()
    assert SOURCE_ONBOARDING_PROBE.is_file()
    assert ONBOARDING_MEASUREMENT_SCRIPT.is_file()
    assert ONBOARDING_MEASUREMENT_SCHEMA.is_file()
    quickstart_documents = (
        readme,
        getting_started,
        distribution,
        onboarding,
    )
    for document in quickstart_documents:
        assert quickstart in document
    assert helper in workflows
    quickstart_script = SOURCE_QUICKSTART_SCRIPT.read_text(encoding='utf-8')
    assert fast_build in quickstart_script
    assert '--base-paths "${REPO_ROOT}"' in quickstart_script
    assert '--packages-select "${EXPECTED_SOURCE_PACKAGES[@]}"' in (
        quickstart_script
    )
    assert '[source-package-inventory-mismatch]' in quickstart_script
    for package in (
        'graph_based_slam',
        'lidarslam',
        'lidarslam_msgs',
        'ndt_omp_ros2',
        'rko_lio',
        'scanmatcher',
    ):
        assert package in quickstart_script
    assert '--repo-only' in quickstart_script
    assert '--dry-run' in quickstart_script
    assert '--json' in quickstart_script
    assert 'source-quickstart-plan-v1.schema.json' in getting_started
    assert 'direct installed command auto-activates this workspace' in (
        quickstart_script
    )
    assert 'no activation step' in quickstart_script
    for document in (readme, getting_started, distribution, workflows):
        assert 'mkdir -p ~/ros2_ws/src' in document

    assert 'auto-activates this build' in readme
    assert 'without changing your shell' in getting_started
    assert 'auto-activates the matching aggregate' in distribution

    assert '8 GiB' in readme
    assert '30 minutes' in readme
    assert '8 GiB' in getting_started
    assert '30 minutes' in getting_started
    assert fast_build not in workflows
    assert 'bash scripts/run_default_ci_checks.sh' in workflows
    assert 'source-route-contract-missing' in onboarding
    assert 'python3 scripts/run_source_onboarding_probe.py' in onboarding
    assert 'python3 scripts/run_candidate_trial.py' in onboarding
    assert '--acknowledge-dedicated-trial-host' in onboarding
    assert 'PREFLIGHT_BLOCKED' in onboarding
    assert 'HARNESS_ERROR' in onboarding
    assert '--public-preflight' in onboarding
    assert '--public-preflight' in SOURCE_ONBOARDING_PROBE.read_text(
        encoding='utf-8'
    )
    assert '--acknowledge-disposable-host' in onboarding
    assert '--acknowledge-isolated-network' in onboarding
    assert '--prompt-human-measurements' in onboarding
    assert '--record-human-measurements-unknown' in onboarding
    assert 'complete_onboarding_measurements.py' in onboarding
    assert '--supplement' in onboarding
    assert 'measurement_supplement_path' in onboarding_trials
    assert '--prompt-active-operator-time' in onboarding
    assert 'source-candidate-not-published' in onboarding
    assert "SOURCE_VERSION='" + VERSION_PATH.read_text(
        encoding='utf-8'
    ).strip() + "'" in onboarding
    assert (
        "SOURCE_COMMIT='549ef03017c776f23fc968881b346aa685356274'"
        in onboarding
    )
    assert onboarding.count('--product-version "$SOURCE_VERSION"') == 3
    assert '--product-version 0.9.0' not in onboarding
    assert '74fe625ab2ee1dc9a0d55ce69bd705d22bac5d76' not in onboarding


def test_custom_pointcloud_lidar_checklist_is_safe_and_copy_ready():
    """The custom-sensor card separates readiness from support claims."""
    workflows = WORKFLOWS_DOC.read_text(encoding='utf-8')

    assert '### Adapting another PointCloud2 LiDAR' in workflows
    for required in (
        'ros2 topic type <POINTCLOUD_TOPIC>',
        'header.frame_id',
        'per-point time field named `t`, `timestamp`,',
        'lidarslam-map doctor /path/to/rosbag2 --json',
        'ros2 topic hz --window 20 <POINTCLOUD_TOPIC>',
        'ros2 run tf2_ros tf2_echo <BASE_FRAME> <LIDAR_FRAME>',
        'scan_period: <SECONDS_PER_SCAN>',
        'scan_min_range: <MIN_RANGE_M>',
        'scan_max_range: <MAX_RANGE_M>',
        'input_cloud:=<POINTCLOUD_TOPIC>',
        'lidar_topic:=<POINTCLOUD_TOPIC>',
        'publish_static_tf:=false',
        'do not attach raw bags, map geometry, or',
        'template=sensor-support.yml',
    ):
        assert required in workflows
    assert 'guessing an extrinsic' in workflows
    assert 'does not validate accuracy' in workflows


def test_g2o_recovery_card_separates_dependency_and_api_failures():
    """The beginner g2o card returns users to the supported package path."""
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    card = getting_started.split(
        '### Recover g2o dependency failures', 1
    )[1].split('## 2. Run the Fixed First-Map Demo', 1)[0]
    normalized = ' '.join(card.split())

    for required in (
        'source /opt/ros/humble/setup.bash',
        'source /opt/ros/jazzy/setup.bash',
        'rosdep resolve libg2o',
        'ros-humble-libg2o',
        'ros-jazzy-libg2o',
        'apt-cache policy "ros-${ROS_DISTRO}-libg2o"',
        'dpkg-query -W',
        'bash scripts/source_quickstart.sh --build-only',
        'product-contract.md#compatibility-and-change-policy',
        'g2o::make_unique',
    ):
        assert required in normalized
    assert 'Foxy and Galactic are end-of-life' in normalized
    assert 'source-built API mismatch' in normalized
    assert 'do not by themselves prove' in normalized
    assert 'do not patch this repository to vendor g2o' in normalized


def test_empty_map_recovery_card_is_copy_ready_and_private():
    """The existing no-map card distinguishes runtime from viewer failure."""
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    card = getting_started.split(
        '### Empty map or viewer: three-check recovery', 1
    )[1].split('For the full operator reference', 1)[0]
    normalized = ' '.join(card.split())

    for required in (
        'lidarslam-map demo ~/ros2_ws --viewer none',
        'ros2 topic hz --window 5 <POINTCLOUD_TOPIC>',
        'header.frame_id <POINTCLOUD_TOPIC>',
        'ros2 run tf2_ros tf2_echo <TF_TARGET_FRAME> <POINTCLOUD_FRAME>',
        'timeout 5s ros2 topic echo --once /map/pointcloud_map',
        'viewer fixed frame to `map`',
        'select `/map/pointcloud_map`',
        'no map, bag, or raw log upload is required',
    ):
        assert required in normalized
    assert normalized.count('Expected:') >= 3
    assert 'replace every angle-bracket placeholder' in normalized


def test_odometry_tf_card_separates_missing_and_stale_transforms():
    """The TF card keeps message frames, path presence, and time distinct."""
    workflows = WORKFLOWS_DOC.read_text(encoding='utf-8')
    card = workflows.split(
        '### Odometry and TF: two separate contracts', 1
    )[1].split('## Run `RKO-LIO + graph_based_slam`', 1)[0]
    normalized = ' '.join(card.split())

    for required in (
        'nav_msgs/msg/Odometry',
        'header.frame_id <ODOM_TOPIC>',
        'child_frame_id <ODOM_TOPIC>',
        'ros2 run tf2_ros tf2_echo <ODOM_FRAME> <BASE_FRAME>',
        'ros2 run tf2_ros tf2_monitor <ODOM_FRAME> <BASE_FRAME>',
        'missing path is a broadcaster/configuration problem',
        'future extrapolation, or stale timestamp is a timing problem',
        'Do not silence TF warnings',
    ):
        assert required in normalized
    assert 'does not, by itself, guarantee' in normalized
    assert (
        'Increasing a lookup timeout alone does not repair stale data'
        in normalized
    )


def test_other_pointcloud_lidar_route_avoids_launch_yaml_forks():
    """Repeated sensor questions should lead to inspection, not file forks."""
    readme = README_PATH.read_text(encoding='utf-8')
    canonical = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    canonical_normalized = ' '.join(canonical.split())
    release = (
        REPO_ROOT / 'docs' / 'releases' / 'v0.9.1.md'
    ).read_text(encoding='utf-8')

    for document in (readme, canonical):
        assert 'Ouster' in document
        assert 'Velodyne' in document
        assert 'RoboSense' in document
        assert 'lidarslam-map doctor /path/to/rosbag2' in document
        assert 'stable reason code' in document
        assert 'maintained profile' in document
        assert 'calibration' in document

    assert (
        "do not edit this package's launch files or YAML"
        in readme
    )
    assert 'forking `lidarslam.launch.py`' in canonical
    assert (
        'does not turn a detected PointCloud2 topic into verified'
        in canonical_normalized
    )
    assert '`doctor` then `start` path' in release
    assert 'detection alone is not a hardware or' in release


def test_map_quality_symptoms_keep_one_bounded_inspection_contract():
    """Visual complaints stay user-reported and use the existing inspector."""
    canonical = AUTOWARE_MAP_AUTHORING.read_text(encoding='utf-8')
    getting_started = GETTING_STARTED.read_text(encoding='utf-8')
    japanese = GETTING_STARTED_JA.read_text(encoding='utf-8')
    product_contract = PRODUCT_CONTRACT_DOC.read_text(encoding='utf-8')
    roadmap = (
        REPO_ROOT / 'docs' / 'roadmap' / '1000-stars.md'
    ).read_text(encoding='utf-8')
    current_packet = (
        REPO_ROOT
        / 'docs'
        / 'evidence'
        / 'growth'
        / 'g0-current-action-packet-2026-08-14.md'
    ).read_text(encoding='utf-8')
    release = (
        REPO_ROOT / 'docs' / 'releases' / 'v0.9.1.md'
    ).read_text(encoding='utf-8')
    cli_contract = json.loads(
        (REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json').read_text(
            encoding='utf-8'
        )
    )
    diagnosis_schema = json.loads(
        DIAGNOSIS_SCHEMA.read_text(encoding='utf-8')
    )

    symptoms = [
        'map-spins-or-spirals',
        'pose-drifts-or-oscillates',
        'map-stops-early',
        'map-is-too-sparse',
        'map-is-not-visible',
    ]
    symptom_contract = cli_contract['map_quality_symptom_triage_contract']
    schema_choices = diagnosis_schema['properties']['symptom_triage'][
        'properties'
    ]['symptom']['enum']

    assert symptom_contract['command'] == 'inspect'
    assert symptom_contract['option'] == '--symptom'
    assert symptom_contract['symptoms'] == symptoms
    assert schema_choices == symptoms
    assert symptom_contract['basis'] == (
        'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED'
    )
    for symptom in symptoms:
        assert symptom in canonical
        assert symptom in japanese
    for document in (canonical, getting_started, japanese):
        assert 'lidarslam-map inspect /path/to/session_bundle' in document
        assert '--symptom' in document
    assert 'not automatic root-cause analysis' in canonical
    assert 'does not automatically identify a root cause' in getting_started
    assert '原因の自動判定' in japanese
    assert 'never edits parameters or starts mapping' in product_contract
    for evidence in (roadmap, current_packet):
        assert 'ee453532a70d2d4b82a6c50c65f19b22d76c239f' in evidence
        assert '9f8a2058a3c702f69d159079568ced8433ee3377' in evidence
        assert '2,474' in evidence
    for evidence in (current_packet, release):
        assert (
            '51c025064de769d1f0c362f51718c52a0beed8492f0881c0e02403b33498e997'
            in evidence
        )


def test_japanese_quickstart_keeps_the_canonical_beginner_contract():
    """The short Japanese route must not drift from supported commands."""
    readme = README_PATH.read_text(encoding='utf-8')
    docs_index = DOCS_INDEX_PATH.read_text(encoding='utf-8')
    japanese = GETTING_STARTED_JA.read_text(encoding='utf-8')
    bundle = (REPO_ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    for command in (
        'lidarslam-map doctor',
        'lidarslam-map doctor /path/to/rosbag2',
        'lidarslam-map start /path/to/rosbag2',
        'lidarslam-map start /path/to/rosbag2 --dry-run',
        'bash scripts/source_quickstart.sh',
        'bash scripts/source_quickstart.sh --dry-run',
        'bash scripts/source_quickstart.sh --dry-run --json',
        'bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2',
        'lidarslam-map demo /path/to/work_dir --resume',
    ):
        assert command in japanese

    for boundary in (
        'ネットワークへ接続せず、ファイルも書きません',
        '517 MB',
        '8 GiB',
        '約30分',
        'PPA/package-manager',
        '未対応',
        'bagはread-onlyでmountされます',
        '`--resume`はmappingを再実行せず',
        '`map_verify: PASS`',
    ):
        assert boundary in japanese

    for other_lidar_boundary in (
        'launch fileやYAMLをfork・編集しないでください',
        '`header.frame_id`',
        'maintained profile',
        'calibrationのreview',
        'hardwareの検証済み対応、または精度保証を意味しません',
    ):
        assert other_lidar_boundary in japanese

    for recovery_entrypoint in (
        'mapまたはviewerが空のとき: 3つの確認',
        'timeout 5s ros2 topic hz --window 5',
        'timeout 5s ros2 topic echo --once --field header.frame_id',
        'ros2 run tf2_ros tf2_echo <TF_TARGET_FRAME> <POINTCLOUD_FRAME>',
        'timeout 5s ros2 topic echo --once /map/pointcloud_map',
        'lidarslam-map inspect /path/to/output --write',
    ):
        assert recovery_entrypoint in japanese

    assert 'ros2 topic echo --once --timeout' not in japanese
    assert '[Getting Started](getting-started.md)' in japanese
    assert '[Operator Workflows](workflows.md)' in japanese
    assert 'docs/getting-started-ja.md' in readme
    assert 'href="getting-started-ja.html"' in docs_index
    assert "'docs/getting-started-ja.md'" in bundle


def test_glim_usability_scorecard_is_neutral_and_release_bundled():
    """The GLIM comparison must keep exact evidence and no-winner policy."""
    document = USABILITY_SCORECARD_DOC.read_text(encoding='utf-8')
    bundle = (REPO_ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )
    checker = REPO_ROOT / 'scripts' / 'check_usability_scorecard.py'
    trial_schema = (
        REPO_ROOT / 'docs' / 'schemas'
        / 'usability-scorecard-trial-v1.schema.json'
    )
    index_schema = (
        REPO_ROOT / 'docs' / 'schemas'
        / 'usability-scorecard-evidence-index-v1.schema.json'
    )
    evidence_index = (
        REPO_ROOT / 'docs' / 'contracts'
        / 'glim-usability-scorecard-evidence-v1.json'
    )

    assert checker.is_file()
    assert trial_schema.is_file()
    assert index_schema.is_file()
    assert evidence_index.is_file()
    for task_id in (
        'discover-supported-path',
        'run-fixed-demo',
        'inspect-own-bag',
        'produce-downstream-artifact',
        'understand-failure',
        'repeat-or-upgrade',
    ):
        assert task_id in document
    assert 'check_usability_scorecard.py --json' in document
    assert '`NOT_READY`' in document
    assert 'does not infer' in document
    assert 'overall winner' in document
    assert "'docs/usability-scorecard.md'" in bundle
    assert "'scripts/check_usability_scorecard.py'" in bundle


def test_repository_root_has_no_ad_hoc_code_or_generated_artifacts():
    """Keep implementation and generated evidence out of the project root."""
    misplaced = sorted(
        path.name
        for path in REPO_ROOT.iterdir()
        if path.is_file() and path.suffix in {'.json', '.py', '.txt'}
    )

    assert misplaced == []
    assert (REPO_ROOT / 'scripts' / 'recovery'
            / 'sota_v2_resume_after_stage_failure.py').is_file()
    assert (REPO_ROOT / 'docs' / 'research'
            / 'project-plan-archive.md').is_file()
    assert (REPO_ROOT / 'docs' / 'research' / 'artifacts'
            / 'scored_summary_fast_track_7point.json').is_file()
