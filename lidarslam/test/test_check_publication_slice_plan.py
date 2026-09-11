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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Tests for the fail-closed local publication slice plan."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_publication_slice_plan.py'
PLAN_PATH = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'g0-publication-slice-plan-2026-08-12.json'
)
SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'publication-slice-plan-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'check_publication_slice_plan',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _load(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _plan() -> dict:
    return copy.deepcopy(_load(PLAN_PATH))


def _schema() -> dict:
    return copy.deepcopy(_load(SCHEMA_PATH))


def _planned_paths(plan: dict) -> list[str]:
    return sorted(
        path
        for review_slice in plan['review_slices']
        for path in review_slice['paths']
    )


def test_tracked_plan_covers_the_exact_candidate_once():
    plan = _plan()
    candidate_ref = plan['whole_pr_review']['follow_up_review']['end_ref']
    actual = CHECKER.committed_paths(
        plan['candidate']['base_sha'], candidate_ref, 'two-dot'
    )

    report = CHECKER.validate_plan(plan, _schema(), actual)

    assert report['status'] == 'PLAN_VALID_LOCAL_ONLY'
    assert report['base_sha'] == '3f4dd70cdc58ad421192559213cdee0bdc41eba8'
    assert report['public_baseline_sha'] == (
        '3ed632e6f6aa1e3ca7f32d893773de1079086ffb'
    )
    assert report['local_tip_sha'] == CHECKER._run_git([
        'rev-parse', candidate_ref,
    ])[0]
    expected_follow_ups = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['public_baseline_sha']}..{candidate_ref}",
    ])[0])
    assert report['follow_up_commit_count'] == expected_follow_ups
    status = CHECKER._run_git(['status', '--short'])
    assert report['worktree_clean'] is (not status)
    assert report['uncommitted_path_count'] == len(status)
    assert report['scope'] == 'worktree-delta-from-pr-base'
    assert report['path_count'] == 349
    assert report['slice_count'] == 7
    assert report['whole_pr_base_sha'] == (
        '86fa9b610c07ccf4d2b0f10939e17c129d34b40a'
    )
    assert report['whole_pr_path_count'] == 396
    assert report['review_phase_count'] == 3
    assert report['review_phase_ids'] == [
        'initial_review',
        'bridge_review',
        'follow_up_review',
    ]
    assert report['review_coverage_complete'] is True
    expected_whole_commits = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['whole_pr_base_sha']}..{candidate_ref}",
    ])[0])
    expected_slice_commits = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['base_sha']}..{candidate_ref}",
    ])[0])
    assert report['whole_pr_commit_count'] == expected_whole_commits
    assert report['initial_review_commit_count'] == 42
    assert report['initial_review_path_count'] == 116
    assert report['bridge_review_commit_count'] == 2
    assert report['bridge_path_count'] == 11
    assert report['follow_up_review_commit_count'] == expected_slice_commits
    assert (
        report['initial_review_commit_count']
        + report['bridge_review_commit_count']
        + report['follow_up_review_commit_count']
        == report['whole_pr_commit_count']
    )
    assert report['overlap_path_count'] == 74
    assert report['overlap_membership_count'] == 80
    assert report['uncovered_path_count'] == 0
    assert report['extraneous_phase_path_count'] == 0
    assert report['merge_commit_count'] == 0
    assert report['remote_mutations_performed'] is False
    assert _planned_paths(plan) == actual


def test_s6_verification_names_every_owned_python_test():
    """The large integration slice must expose each owned focused test."""
    review_slice = next(
        item
        for item in _plan()['review_slices']
        if item['id'] == 'S6-product-shell-integration'
    )
    commands = '\n'.join(review_slice['verification'])
    owned_tests = [
        path
        for path in review_slice['paths']
        if '/test/' in path and path.endswith('.py')
    ]

    assert owned_tests
    assert [path for path in owned_tests if path not in commands] == []


def test_unassigned_candidate_path_is_rejected():
    plan = _plan()
    actual = _planned_paths(plan) + ['scripts/unplanned_follow_up.py']

    with pytest.raises(CHECKER.PlanError, match='missing_from_plan'):
        CHECKER.validate_plan(plan, _schema(), actual)


def test_stale_planned_path_is_rejected():
    plan = _plan()
    actual = _planned_paths(plan)
    actual.remove(plan['review_slices'][0]['paths'][0])

    with pytest.raises(CHECKER.PlanError, match='absent_from_candidate'):
        CHECKER.validate_plan(plan, _schema(), actual)


def test_duplicate_path_ownership_is_rejected():
    plan = _plan()
    duplicate = plan['review_slices'][0]['paths'][0]
    plan['review_slices'][1]['paths'].append(duplicate)
    plan['review_slices'][1]['paths'].sort()

    with pytest.raises(CHECKER.PlanError, match='multiple slices'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(_plan()))


def test_later_slice_dependency_is_rejected():
    plan = _plan()
    plan['review_slices'][0]['depends_on'] = [plan['review_slices'][1]['id']]

    with pytest.raises(CHECKER.PlanError, match='unknown or later'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_non_consecutive_order_is_rejected():
    plan = _plan()
    plan['review_slices'][-1]['order'] = 9

    with pytest.raises(CHECKER.PlanError, match='consecutive'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_stale_inventory_digest_is_rejected():
    plan = _plan()
    plan['candidate']['expected_paths_sha256'] = '0' * 64

    with pytest.raises(CHECKER.PlanError, match='sha256 is stale'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_unresolvable_public_head_is_rejected():
    plan = _plan()
    plan['candidate']['public_head_sha'] = '0' * 40

    with pytest.raises(CHECKER.PlanError, match='lineage cannot be verified'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_bridge_allowlist_drift_is_rejected():
    plan = _plan()
    plan['whole_pr_review']['bridge_review']['allowed_paths'].pop()

    with pytest.raises(CHECKER.PlanError, match='allowlist mismatch'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_stale_bridge_digest_is_rejected():
    plan = _plan()
    plan['whole_pr_review']['bridge_review'][
        'expected_paths_sha256'
    ] = '0' * 64

    with pytest.raises(
        CHECKER.PlanError,
        match='bridge review.*sha256 is stale',
    ):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_non_contiguous_review_phases_are_rejected():
    plan = _plan()
    plan['whole_pr_review']['bridge_review']['start_sha'] = '0' * 40

    with pytest.raises(CHECKER.PlanError, match='not contiguous'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_stale_whole_pr_digest_is_rejected():
    plan = _plan()
    plan['whole_pr_review']['expected_paths_sha256'] = '0' * 64

    with pytest.raises(CHECKER.PlanError, match='whole-PR.*sha256 is stale'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_uncovered_whole_pr_path_is_rejected():
    plan = _plan()
    uncovered_path = '.gitattributes'
    owner = next(
        review_slice
        for review_slice in plan['review_slices']
        if uncovered_path in review_slice['paths']
    )
    owner['paths'].remove(uncovered_path)
    actual = _planned_paths(plan)
    plan['candidate']['expected_path_count'] = len(actual)
    plan['candidate']['expected_paths_sha256'] = (
        CHECKER.path_inventory_sha256(actual)
    )

    with pytest.raises(CHECKER.PlanError, match='uncovered='):
        CHECKER.validate_plan(plan, _schema(), actual)


def test_unsafe_whole_pr_review_record_is_rejected():
    plan = _plan()
    plan['whole_pr_review']['follow_up_review'][
        'review_record'
    ] = '../outside.md'

    with pytest.raises(CHECKER.PlanError, match='unsafe repository-relative'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_review_record_outside_its_phase_is_rejected():
    plan = _plan()
    plan['whole_pr_review']['initial_review'][
        'review_record'
    ] = 'docs/evidence/growth/g0-pr-review-coverage-2026-08-17.md'

    with pytest.raises(CHECKER.PlanError, match='outside the initial review'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_plan_cannot_authorize_github_writes():
    plan = _plan()
    plan['authority']['github_writes_authorized'] = True

    with pytest.raises(CHECKER.PlanError, match='schema validation'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_source_quickstart_verification_is_self_contained_and_read_only():
    plan = _plan()
    source_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S4-source-onboarding'
    )
    command = next(
        item
        for item in source_slice['verification']
        if 'source_quickstart.sh --dry-run' in item
    )

    result = subprocess.run(
        ['bash', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    workspace_line = next(
        line
        for line in result.stdout.splitlines()
        if line.startswith('  Workspace: ')
    )
    workspace = Path(workspace_line.removeprefix('  Workspace: '))
    assert not workspace.exists()
    assert 'Commands (--dry-run; nothing executed)' in result.stdout


def test_ros_bag_verification_sources_ros_from_an_unsourced_shell():
    """The S3 copy-ready command must restore its own ROS Python imports."""
    plan = _plan()
    lifecycle_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S3-map-lifecycle'
    )
    command = next(
        item
        for item in lifecycle_slice['verification']
        if 'test_sensor_setup_wizard.py' in item
    )
    environment = os.environ.copy()
    for name in (
        'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'LD_LIBRARY_PATH',
        'PYTHONPATH',
        'ROS_PYTHON_VERSION',
        'ROS_VERSION',
    ):
        environment.pop(name, None)

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=90,
    )

    assert command.startswith(CHECKER.ROS_SOURCE_PREFIX)
    assert result.returncode == 0, result.stderr
    assert 'passed in' in result.stdout


def test_product_shell_verification_sources_ros_from_an_unsourced_shell():
    """The S6 copy-ready command must restore its ROS bag imports."""
    plan = _plan()
    product_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S6-product-shell-integration'
    )
    command = next(
        item
        for item in product_slice['verification']
        if 'test_lidarslam_product_cli.py' in item
    )
    environment = os.environ.copy()
    for name in (
        'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'LD_LIBRARY_PATH',
        'PYTHONPATH',
        'ROS_PYTHON_VERSION',
        'ROS_VERSION',
    ):
        environment.pop(name, None)

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=90,
    )

    assert command.startswith(CHECKER.ROS_SOURCE_PREFIX)
    assert result.returncode == 0, result.stderr
    assert 'passed in' in result.stdout


def test_mixed_package_pytest_process_is_rejected():
    """Known duplicate module names require one pytest process per package."""
    plan = _plan()
    plan['review_slices'][4]['verification'].append(
        'python3 -m pytest -q -p no:cacheprovider '
        'lidarslam/test/test_benchmark_summary_profiles.py '
        'graph_based_slam/test/test_ntu_viral_download_script.py'
    )

    with pytest.raises(CHECKER.PlanError, match='separate processes'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_ros_dependent_command_without_source_is_rejected():
    """A valid plan cannot silently depend on the caller's shell state."""
    plan = _plan()
    command = plan['review_slices'][2]['verification'][0]
    plan['review_slices'][2]['verification'][0] = command.removeprefix(
        CHECKER.ROS_SOURCE_PREFIX
    )

    with pytest.raises(CHECKER.PlanError, match='source ROS first'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_colcon_test_requires_a_prior_build_of_every_selected_package():
    """A clean checkout must not rely on an unmentioned prior build."""
    plan = _plan()
    runtime_slice = plan['review_slices'][0]
    command = next(
        item for item in runtime_slice['verification']
        if 'colcon test --packages-select' in item
    )
    runtime_slice['verification'][
        runtime_slice['verification'].index(command)
    ] = command.replace(
        'colcon build --packages-up-to graph_based_slam scanmatcher && ',
        '',
    )

    with pytest.raises(CHECKER.PlanError, match='build all tested packages'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))

    plan = _plan()
    runtime_slice = plan['review_slices'][0]
    command = next(
        item for item in runtime_slice['verification']
        if 'colcon test --packages-select' in item
    )
    runtime_slice['verification'][
        runtime_slice['verification'].index(command)
    ] = f'{command} "'

    with pytest.raises(CHECKER.PlanError, match='invalid shell quoting'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))

    plan = _plan()
    runtime_slice = plan['review_slices'][0]
    command = next(
        item for item in runtime_slice['verification']
        if 'colcon test --packages-select' in item
    )
    runtime_slice['verification'][
        runtime_slice['verification'].index(command)
    ] = command.replace(
        '--packages-up-to graph_based_slam scanmatcher',
        '--packages-up-to graph_based_slam',
    )

    with pytest.raises(CHECKER.PlanError, match='build all tested packages'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_product_shell_command_without_source_is_rejected():
    """S6 cannot silently depend on ROS state inherited by the caller."""
    plan = _plan()
    product_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S6-product-shell-integration'
    )
    index = next(
        index
        for index, item in enumerate(product_slice['verification'])
        if 'test_lidarslam_product_cli.py' in item
    )
    product_slice['verification'][index] = product_slice['verification'][
        index
    ].removeprefix(CHECKER.ROS_SOURCE_PREFIX)

    with pytest.raises(CHECKER.PlanError, match='source ROS first'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_pytest_cache_and_remote_write_commands_are_rejected():
    """Review commands remain local-only and avoid untracked cache writes."""
    plan = _plan()
    plan['review_slices'][0]['verification'][0] = (
        'python3 -m pytest -q graph_based_slam/test/'
        'test_mid360_robot_tools.py -k frame'
    )
    with pytest.raises(CHECKER.PlanError, match='disable its cache'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))

    plan = _plan()
    plan['review_slices'][-1]['verification'].append(
        'gh pr ready 427'
    )
    with pytest.raises(CHECKER.PlanError, match='remote write'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_cli_emits_a_machine_readable_local_only_report():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['status'] == 'PLAN_VALID_LOCAL_ONLY'
    assert report['public_baseline_sha'] == (
        '3ed632e6f6aa1e3ca7f32d893773de1079086ffb'
    )
    assert report['local_tip_sha'] == _plan()[
        'whole_pr_review'
    ]['follow_up_review']['end_ref']
    candidate_ref = report['local_tip_sha']
    expected_follow_ups = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['public_baseline_sha']}..{candidate_ref}",
    ])[0])
    assert report['follow_up_commit_count'] == expected_follow_ups
    status = CHECKER._run_git(['status', '--short'])
    assert report['worktree_clean'] is (not status)
    assert report['uncommitted_path_count'] == len(status)
    assert report['path_count'] == 349
    assert report['whole_pr_path_count'] == 396
    assert report['whole_pr_commit_count'] == int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['whole_pr_base_sha']}..{candidate_ref}",
    ])[0])
    assert report['review_phase_count'] == 3
    assert report['review_coverage_complete'] is True
    assert report['bridge_path_count'] == 11
    assert report['uncovered_path_count'] == 0
    assert report['merge_commit_count'] == 0
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_slice_json_binds_exact_scope_without_executing_commands():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--slice',
            'S1-runtime-safety',
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    review_slice = report['review_slice']
    assert report['status'] == 'SLICE_REVIEW_READY_LOCAL_ONLY'
    assert review_slice['id'] == 'S1-runtime-safety'
    assert review_slice['order'] == 1
    assert report['candidate']['slice_count'] == 7
    assert report['candidate']['whole_pr_path_count'] == 396
    assert report['candidate']['review_phase_count'] == 3
    assert report['candidate']['review_coverage_complete'] is True
    assert report['candidate']['bridge_path_count'] == 11
    assert report['candidate']['uncommitted_path_count'] >= 0
    assert review_slice['path_count'] == 16
    assert review_slice['depends_on'] == []
    assert review_slice['publication_gate'] == 'PUBLIC_CI'
    budget = review_slice['review_budget']
    assert budget['path_count'] == review_slice['path_count']
    assert budget['text_path_count'] + budget['binary_path_count'] == (
        budget['path_count']
    )
    assert budget['unresolved_path_count'] == 0
    assert budget['additions'] >= 0
    assert budget['deletions'] >= 0
    assert budget['changed_lines'] == (
        budget['additions'] + budget['deletions']
    )
    assert 1 <= len(budget['hotspots']) <= 3
    assert all(
        hotspot['path'] in review_slice['paths']
        for hotspot in budget['hotspots']
    )
    assert report['commands_executed'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_slice_human_card_has_one_safe_next_action():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--slice', 'S7-publication-control'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert '# S7-publication-control:' in result.stdout
    assert '- Commands executed by this card: no' in result.stdout
    assert '- GitHub write authorized: no' in result.stdout
    assert '- Worktree clean:' in result.stdout
    assert '- Uncommitted paths:' in result.stdout
    assert '- Whole-PR paths: 396' in result.stdout
    assert '- Whole-PR review coverage complete: yes' in result.stdout
    assert '- CI bridge paths: 11' in result.stdout
    assert '- Text delta: +' in result.stdout
    assert '- Binary paths: ' in result.stdout
    assert 'Review hotspots (largest textual deltas):' in result.stdout
    assert result.stdout.count('Next action:') == 1


def test_overview_json_binds_all_phases_and_slices_without_writes():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--overview', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    candidate = report['candidate']
    assert report['status'] == 'PR_REVIEW_OVERVIEW_READY_LOCAL_ONLY'
    assert candidate['repository'] == 'rsasaki0109/lidar_slam_ros2'
    assert candidate['pull_request'] == 427
    assert candidate['local_tip_sha'] == _plan()[
        'whole_pr_review'
    ]['follow_up_review']['end_ref']
    assert candidate['whole_pr_path_count'] == 396
    assert candidate['follow_up_path_count'] == 349
    assert candidate['review_coverage_complete'] is True
    assert candidate['uncovered_path_count'] == 0
    assert candidate['extraneous_phase_path_count'] == 0
    assert candidate['merge_commit_count'] == 0
    whole_budget = candidate['whole_pr_review_budget']
    assert whole_budget['path_count'] == candidate['whole_pr_path_count']
    assert whole_budget['unresolved_path_count'] == 0
    assert whole_budget['binary_path_count'] == 2
    assert whole_budget['binary_paths'] == [
        'lidarslam/images/social_autoware_map_authoring.png',
        'lidarslam/images/social_autoware_map_authoring_demo.mp4',
    ]
    assert whole_budget['unresolved_paths'] == []
    assert whole_budget['changed_lines'] == (
        whole_budget['additions'] + whole_budget['deletions']
    )
    assert [item['id'] for item in report['review_phases']] == [
        'P0-initial-review',
        'P1-ci-bridge',
        'P2-follow-up-slices',
    ]
    assert [item['commit_count'] for item in report['review_phases'][:2]] == [
        42,
        2,
    ]
    assert sum(
        item['commit_count'] for item in report['review_phases']
    ) == candidate['whole_pr_commit_count']
    assert [item['order'] for item in report['review_slices']] == list(
        range(1, 8)
    )
    assert sum(
        item['path_count'] for item in report['review_slices']
    ) == candidate['follow_up_path_count']
    follow_up_budget = report['review_phases'][2]['review_budget']
    for field in (
        'path_count',
        'text_path_count',
        'binary_path_count',
        'unresolved_path_count',
        'additions',
        'deletions',
        'changed_lines',
    ):
        assert sum(
            item['review_budget'][field]
            for item in report['review_slices']
        ) == follow_up_budget[field]
    assert follow_up_budget['unresolved_path_count'] == 0
    assert follow_up_budget['binary_path_count'] == 2
    assert all(
        len(item['review_budget']['hotspots']) <= 3
        for item in report['review_slices']
    )
    assert report['commands_executed'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_s6_review_budget_names_binary_assets_for_manual_review():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--slice',
            'S6-product-shell-integration',
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    budget = json.loads(result.stdout)['review_slice']['review_budget']
    assert budget['binary_path_count'] == 2
    assert budget['binary_paths'] == [
        'lidarslam/images/social_autoware_map_authoring.png',
        'lidarslam/images/social_autoware_map_authoring_demo.mp4',
    ]
    assert budget['unresolved_paths'] == []


def test_overview_human_card_is_bounded_and_copy_ready():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--overview'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert '# PR #427 review overview' in result.stdout
    assert '- Whole-PR paths: 396' in result.stdout
    assert '- Follow-up paths: 349' in result.stdout
    assert '- Whole-PR review coverage complete: yes' in result.stdout
    assert '- Whole-PR text delta: +' in result.stdout
    assert '- Whole-PR binary paths: 2' in result.stdout
    assert '| Text delta | Binary | Largest text path |' in result.stdout
    assert '| P0-initial-review |' in result.stdout
    assert '| P1-ci-bridge |' in result.stdout
    assert '| P2-follow-up-slices |' in result.stdout
    for slice_id in (
        'S1-runtime-safety',
        'S2-first-map-foundation',
        'S3-map-lifecycle',
        'S4-source-onboarding',
        'S5-distribution-readiness',
        'S6-product-shell-integration',
        'S7-publication-control',
    ):
        assert f'| {slice_id} |' in result.stdout
    assert '<ID>' in result.stdout
    assert '- Commands executed by this card: no' in result.stdout
    assert '- GitHub write authorized: no' in result.stdout
    assert result.stdout.count('Next action:') == 1


def test_malformed_git_numstat_fails_closed(monkeypatch):
    monkeypatch.setattr(
        CHECKER,
        '_run_git',
        lambda arguments: ['not-a-numstat-row'],
    )

    with pytest.raises(CHECKER.PlanError, match='malformed Git numstat row'):
        CHECKER._git_diff_numstat('a' * 40, 'b' * 40, 'two-dot')


def test_overview_and_slice_modes_are_mutually_exclusive():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--overview',
            '--slice',
            'S1-runtime-safety',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'not allowed with argument' in result.stderr


def test_unknown_slice_fails_closed_and_lists_valid_ids():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--slice', 'S8-not-real', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'PLAN_INVALID'
    assert 'unknown review slice' in report['error']
    assert 'S1-runtime-safety' in report['error']
    assert report['remote_mutations_performed'] is False
