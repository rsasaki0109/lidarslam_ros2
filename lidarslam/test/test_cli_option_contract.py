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

"""Keep the published CLI option inventory synchronized with command help."""

from __future__ import annotations

import json
from pathlib import Path
import re
import runpy
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
CONTRACT_PATH = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
PROFILE_REGISTRY_PATH = REPO_ROOT / 'scripts' / 'product_profiles.py'
OPTION_PATTERN = re.compile(
    r'(?<![A-Za-z0-9-])(--[a-z][a-z0-9-]*|-h)(?![A-Za-z0-9-])'
)
VALUE_OPTION_PATTERN = re.compile(
    r'(?P<option>--[a-z][a-z0-9-]*)[ =]'
    r'(?P<value>\{[^}\n]+\}|<[^>\n]+>|[A-Z][A-Z0-9_]*)'
)


def _contract() -> dict[str, object]:
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def _option_names(entries: list[dict[str, object]]) -> set[str]:
    return {
        name
        for entry in entries
        for name in entry['names']
    }


def _help(command: str, *, all_options: bool = False) -> str:
    help_option = '--help-all' if all_options else '--help'
    completed = subprocess.run(
        [str(CLI), command, help_option],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


def _rendered_value_options(rendered_help: str) -> dict[str, str]:
    option_lines = '\n'.join(
        line
        for line in rendered_help.splitlines()
        if line.lstrip().startswith('-')
    )
    return {
        match.group('option'): match.group('value')
        for match in VALUE_OPTION_PATTERN.finditer(option_lines)
    }


def test_contract_identifies_the_complete_product_surface():
    """The manifest enumerates every product command and global flag."""
    contract = _contract()

    assert contract['schema_version'] == 1
    assert contract['contract_id'] == 'lidarslam-map-v1'
    assert contract['product_command'] == 'lidarslam-map'
    assert contract['exit_codes'] == {
        '0': 'command completed successfully',
        '2': 'invalid usage, input, profile, or output path',
        '70': 'internal or tooling error prevented command startup',
        'other_nonzero': 'delegated workflow or viewer failure',
    }
    activation = contract['environment_activation_contract']
    assert activation['direct_launchers'] == [
        'absolute installed lidarslam-map path',
        'repo-local ./scripts/lidarslam after a matching workspace build',
    ]
    assert 'nearest aggregate setup.bash' in activation['behavior']
    assert 'curated lidarslam_cli.py' in activation['matching_rules'][0]
    assert 'curated installed lidarslam_cli.py' in (
        activation['matching_rules'][1]
    )
    assert 'unrelated parent workspace' in activation['matching_rules'][2]
    assert 'does not modify' in activation['shell_rule']
    assert 'normal PATH activation' in activation['normal_path_rule']
    home = contract['interactive_home_contract']
    assert home['routes'] == [
        'demo',
        'start',
        'sessions',
        'doctor',
        'help',
    ]
    assert 'interactive stdin and stdout' in home['trigger']
    assert 'usage exit code 2' in home['non_interactive_behavior']
    assert 'explicit yes' in home['safety_rules'][2]
    assert 'sensor and calibration review' in home['safety_rules'][3]
    assert 'uses no network' in home['safety_rules'][4]
    assert 'starts no delegated command' in home['safety_rules'][5]
    doctor = contract['system_doctor_contract']
    assert doctor['schema_uri'].endswith(
        '/schemas/system-doctor-v1.schema.json'
    )
    assert doctor['public_evidence_schema_uri'].endswith(
        '/schemas/public-doctor-evidence-v1.schema.json'
    )
    assert doctor['command'] == 'doctor'
    assert doctor['statuses'] == ['ready', 'action_required']
    assert doctor['finding_fields'] == ['code', 'message', 'next_action']
    assert 'omit rosbag2_dir' in doctor['system_mode']
    assert 'maintained-profile preflight' in doctor['bag_mode']
    assert 'no network and writes no files' in doctor['safety_rules'][0]
    assert 'omits checkout' in doctor['safety_rules'][1]
    assert '--public-json requires a bag' in doctor['safety_rules'][4]
    assert 'bag-preflight-input-error' in doctor['safety_rules'][5]
    assert 'every human bag report warns' in doctor['safety_rules'][6]
    assert 'shell-safe --public-json command' in doctor['safety_rules'][6]
    assert 'exact-input start command' in doctor['safety_rules'][7]
    assert 'hides lower-level mapping alternatives' in doctor['safety_rules'][7]
    assert 'with findings' in doctor['safety_rules'][8]
    assert 'withholds start' in doctor['safety_rules'][8]
    assert 'exact-input doctor retry' in doctor['safety_rules'][8]
    assert 'product human bag card is bounded' in doctor['safety_rules'][9]
    assert 'without topic or frame names' in doctor['safety_rules'][9]
    assert 'first finding' in doctor['safety_rules'][9]
    assert 'exact-input --json command' in doctor['safety_rules'][10]
    assert 'expert human report' in doctor['safety_rules'][10]
    symptom = contract['map_quality_symptom_triage_contract']
    assert symptom['schema_uri'].endswith('/schemas/diagnosis-v1.schema.json')
    assert symptom['command'] == 'inspect'
    assert symptom['option'] == '--symptom'
    assert symptom['symptoms'] == [
        'map-spins-or-spirals',
        'pose-drifts-or-oscillates',
        'map-stops-early',
        'map-is-too-sparse',
        'map-is-not-visible',
    ]
    assert symptom['basis'] == (
        'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED'
    )
    assert 'existing retained-run inspector' in symptom['safety_rules'][0]
    assert 'without editing parameters' in symptom['safety_rules'][1]
    assert 'never convert' in symptom['safety_rules'][3]
    support = contract['support_bundle_contract']
    assert any(
        'reported visual symptom only as one fixed code' in rule
        for rule in support['privacy_rules']
    )
    assert any(
        'automatically attributed reported-symptom claim' in rule
        for rule in support['evidence_rules']
    )
    demo = contract['first_map_demo_plan_contract']
    assert demo['schema_uri'].endswith(
        '/schemas/first-map-demo-plan-v1.schema.json'
    )
    assert demo['command'] == 'demo'
    assert demo['statuses'] == [
        'ready',
        'resume_ready',
        'already_verified',
        'not_ready',
    ]
    assert 'checksum-pinned' in demo['dataset_rules'][0]
    assert 'writes nothing' in demo['safety_rules'][0]
    assert any(
        're-hash the registered archive' in rule
        for rule in demo['safety_rules']
    )
    assert any(
        'receipt-bound PASS' in rule
        for rule in demo['safety_rules']
    )
    assert any(
        'never restart mapping through resume' in rule
        for rule in demo['safety_rules']
    )
    assert demo['viewer_rule'].startswith('viewer failure never replaces')
    assert demo['json_rule'].startswith('--json requires --dry-run')
    assert contract['sensor_setup_rejection_contract'] == {
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/sensor-setup-rejection-v1.schema.json'
        ),
        'commands': ['start', 'setup'],
        'exit_code': 2,
        'status': 'not_ready',
        'files_written': False,
        'reason_codes': [
            'no-maintained-profile',
            'profile-incompatible',
        ],
        'finding_fields': ['code', 'message', 'next_action'],
        'operator_rule': (
            'key on stable codes; display messages and next actions'
        ),
    }
    assert contract['sensor_setup_review_contract'] == {
        'commands': ['start', 'setup'],
        'status': 'review_required',
        'safety_rules': [
            'live interactive start renders one calibration review then asks '
            'one fail-closed confirmation without presenting a second --yes '
            'command',
            'non-interactive start, setup, and dry-run retain the exact '
            'reviewed rerun command',
            'decline, EOF, or unreviewed calibration starts no mapping',
        ],
    }
    assert contract['sensor_setup_ready_contract'] == {
        'commands': ['start', 'setup'],
        'statuses': ['ready', 'dry_run'],
        'display_rules': [
            'confirmed live start skips the repeated setup review and proceeds '
            'directly to its map start and progress card',
            'setup and dry-run retain the complete selected input, '
            'calibration, and map command review',
            'the compact live handoff does not alter sensor_setup.json, '
            'session index, progress, or recovery contracts',
        ],
    }
    recovery = contract['map_session_recovery_contract']
    assert recovery['schema_uri'].endswith(
        '/schemas/map-session-recovery-v1.schema.json'
    )
    assert recovery['command'] == 'start'
    assert recovery['status'] == 'action_required'
    assert recovery['artifact'] == 'map_session_recovery.json'
    assert recovery['browser_artifact'] == 'session.html'
    assert recovery['browser_behavior'] == (
        'render action_required in the common session page; open for start '
        '--viewer browser and retain without opening for --viewer none'
    )
    assert recovery['browser_rules'] == [
        'self-contained HTML with no network resources',
        'escape bag paths, diagnosis text, evidence paths, and commands',
        (
            'an HTML write failure must not suppress session.json or the '
            'JSON recovery contract'
        ),
        'opening failure must not replace the delegated map exit code',
    ]
    assert set(recovery['reason_codes']) == {
        'ambiguous-output-state',
        'run-manifest-unreadable',
        'workflow-state-uncertain',
        'postprocessing-incomplete',
        'storage-exhausted',
        'workflow-interrupted',
        'ros-parameters-invalid',
        'tf-messages-invalid',
        'tf-tree-disconnected',
        'map-save-failed',
        'ros-node-died',
        'gnss-constraints-missing',
        'map-verification-failed',
        'runner-start-failed',
        'map-output-incomplete',
        'workflow-failed',
    }
    assert recovery['finding_fields'] == ['code', 'message', 'next_action']
    assert recovery['safety_rules'] == [
        'preserve setup and run evidence',
        (
            'resume terminal post-processing only when manifest-v2 proves '
            'it is safe'
        ),
        'retry the pinned setup only into a fresh output directory',
        'one Ctrl-C makes the stable CLI wait for the start helper, signals '
        'the complete isolated delegated process group, waits up to 20 '
        'seconds for cleanup and terminal evidence before requesting '
        'termination, waits 10 more seconds before a forced group reap, and '
        'then renders the same one-action recovery handoff without a Python '
        'traceback, false timeout label, or recent-launch-log dump',
    ]
    assert recovery['terminal_rules'] == [
        'render the first reason, any remaining stable finding codes, exactly '
        'one next_command, and one retained detail path',
        'do not repeat per-finding actions, retry alternatives, or inspect '
        'alternatives in the default terminal card',
        'retain every finding, action, retry, inspect command, and evidence '
        'path in map_session_recovery.json and the derived session handoff',
    ]
    assert recovery['operator_rule'] == (
        'run next_command first; key automation on reason.code'
    )
    session = contract['map_session_index_contract']
    assert session['schema_uri'].endswith(
        '/schemas/map-session-index-v1.schema.json'
    )
    assert session['artifact'] == 'session.json'
    assert session['browser_artifact'] == 'session.html'
    assert session['statuses'] == [
        'running',
        'verified',
        'unverified',
        'action_required',
    ]
    assert session['progress_rules'] == [
        'durable lifecycle stage changes update session.json and session.html '
        'and print the new stage',
        'an unchanged live stage prints at most one terminal heartbeat every '
        '30 seconds with its label and monotonic elapsed time but performs no '
        'session artifact write',
        'heartbeats report neither percentage nor ETA and do not claim that '
        'the delegated workflow made forward progress',
    ]
    assert session['action_rule'] == (
        'for terminal statuses, actions[0] is the recommended copy-ready '
        'next action'
    )
    assert session['quality_rules'][0] == (
        'summarize workflow, map output, Autoware verification, and evidence '
        'integrity without inventing a numeric score'
    )
    assert 'not_verified' in session['quality_rules'][2]
    assert 'semantically incomplete' in session['quality_rules'][3]
    assert session['terminal_rules'] == [
        'a successful start renders one VERIFIED or UNVERIFIED completion '
        'card instead of separate completion and summary blocks',
        'the completion card contains map output, verification, viewer, '
        'session index, session page, evidence paths, and exactly one '
        'recommended Next command',
        'viewer failure makes the single Next action the view retry, adds one '
        'warning, and does not change verified map status',
        'session-index generation failure retains a completed fallback card '
        'with the map output and one view command',
    ]
    catalog = contract['map_session_catalog_contract']
    assert catalog['schema_uri'].endswith(
        '/schemas/map-session-catalog-v1.schema.json'
    )
    assert catalog['command'] == 'sessions'
    assert catalog['browser_artifact'] == 'sessions.html'
    assert catalog['json_rule'].startswith('--json is read-only')
    assert 'direct child bundles only' in catalog['scan_rules'][0]
    assert 'symlinks' in catalog['scan_rules'][1]
    assert '2 MiB' in catalog['scan_rules'][2]
    comparison = contract['map_session_comparison_contract']
    assert comparison['schema_uri'].endswith(
        '/schemas/map-session-comparison-v1.schema.json'
    )
    assert comparison['command'] == 'compare'
    assert comparison['row_count'] == 14
    assert comparison['json_rule'].startswith('--json is read-only')
    assert 'numeric score' in comparison['decision_rules'][0]
    assert 'winner' in comparison['decision_rules'][1]
    assert 'unavailable' in comparison['evidence_rules'][2]
    support = contract['support_bundle_contract']
    assert support['schema_uri'].endswith(
        '/schemas/support-bundle-v1.schema.json'
    )
    assert support['first_map_handoff_schema_uri'].endswith(
        '/schemas/first-map-handoff-v1.schema.json'
    )
    assert support['command'] == 'support'
    assert support['first_map_command'] == 'report'
    assert support['legacy_first_map_command'] == 'support --first-map'
    assert support['archive_members'] == [
        'README.txt',
        'issue-body.md',
        'support-report.json',
    ]
    assert 'credential-like' in support['privacy_rules'][1]
    assert 'review_before_sharing' in support['privacy_rules'][3]
    assert 'never follow' in support['evidence_rules'][2]
    assert support['json_rule'].startswith('--json is read-only')
    assert support['first_map_handoff_rules'][0].startswith(
        'report is the read-only first-map command'
    )
    assert 'revalidate' in support['first_map_handoff_rules'][2]
    assert 'copy-ready PASS result' in support['first_map_handoff_rules'][3]
    assert 'safe environment hints' in support['first_map_handoff_rules'][3]
    assert 'never the public attachment' in support['first_map_handoff_rules'][3]
    assert 'remaining operator-supplied fields' in (
        support['first_map_handoff_rules'][3]
    )
    assert 'never upload' in support['first_map_handoff_rules'][4]
    assert 'never upload' in support['write_rules'][2]
    assert set(contract['commands']) == {
        'demo',
        'start',
        'sessions',
        'compare',
        'report',
        'support',
        'doctor',
        'setup',
        'run',
        'inspect',
        'view',
        'edit',
        'merge',
        'migrate-manifest',
        'rollback-plan',
    }
    assert _option_names(contract['global_options']) == {
        '-h',
        '--help',
        '--help-all',
        '--version',
    }


def test_recovery_commands_are_kept_out_of_beginner_help():
    """Recovery tools should be explicit without enlarging the golden path."""
    normal = subprocess.run(
        [str(CLI), '--help'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    complete = subprocess.run(
        [str(CLI), '--help-all'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert normal.returncode == 0
    assert complete.returncode == 0
    assert 'no command in an interactive terminal' in normal.stdout
    for command in ('migrate-manifest', 'rollback-plan'):
        assert command not in normal.stdout
        assert command in complete.stdout


def test_each_subcommand_help_matches_its_option_inventory():
    """Both help levels should match the machine-readable visibility rules."""
    contract = _contract()
    exclusions = contract['help_modes']['normal']['excludes']

    for command, command_contract in contract['commands'].items():
        rendered_all = _help(command, all_options=True)
        all_option_lines = '\n'.join(
            line
            for line in rendered_all.splitlines()
            if line.lstrip().startswith('-')
        )
        rendered_all_options = set(OPTION_PATTERN.findall(all_option_lines))
        contract_options = _option_names(command_contract['options'])

        assert rendered_all_options == contract_options, command
        assert command_contract['positional']['name'] in rendered_all

        rendered_normal = _help(command)
        normal_option_lines = '\n'.join(
            line
            for line in rendered_normal.splitlines()
            if line.lstrip().startswith('-')
        )
        rendered_normal_options = set(
            OPTION_PATTERN.findall(normal_option_lines)
        )
        expected_normal_options = _option_names([
            entry
            for entry in command_contract['options']
            if (
                entry['stability'] not in exclusions['stability']
                and entry['tier'] not in exclusions['tiers']
            )
        ])
        assert rendered_normal_options == expected_normal_options, command


def test_stability_and_tier_values_are_explicit():
    """Every flag should carry a bounded stability state and operator tier."""
    contract = _contract()
    entries = list(contract['global_options'])
    for command in contract['commands'].values():
        entries.extend(command['options'])
        assert command['positional']['stability'] == 'stable'

    assert {entry['stability'] for entry in entries} <= {
        'stable',
        'provisional',
        'deprecated',
    }
    assert all(entry['tier'] for entry in entries)
    deprecated = [
        entry for entry in entries if entry['stability'] == 'deprecated'
    ]
    assert deprecated
    assert all(entry['replacement'] for entry in deprecated)


def test_value_contract_matches_full_help_and_bounded_choices():
    """Value-taking options must publish their type, default and metavar."""
    contract = _contract()

    assert set(contract['value_options']) == set(contract['commands'])
    for command, command_contract in contract['commands'].items():
        value_contract = contract['value_options'][command]
        rendered_values = _rendered_value_options(
            _help(command, all_options=True)
        )
        assert set(value_contract) == set(rendered_values), command

        declared_options = _option_names(command_contract['options'])
        for option, value in value_contract.items():
            assert option in declared_options
            assert value['kind'] in {
                'directory',
                'enum',
                'frame',
                'integer',
                'number',
                'file',
                'transform',
            }
            assert 'default' in value
            assert value['value_name'] == rendered_values[option]
            if value['kind'] == 'enum':
                assert value['choices']
                if value['value_name'].startswith('{'):
                    expected = '{' + ','.join(value['choices']) + '}'
                    assert expected == value['value_name']
            else:
                assert 'choices' not in value


def test_profile_option_and_help_use_the_maintained_registry():
    """Keep profile choices and descriptions in one authoritative source."""
    contract = _contract()
    registry = runpy.run_path(str(PROFILE_REGISTRY_PATH))
    profile_ids = tuple(registry['PROFILE_IDS'])
    profile_help = tuple(registry['PROFILE_HELP'])

    assert profile_ids
    assert len(profile_ids) == len(set(profile_ids))
    assert tuple(
        contract['value_options']['run']['--profile']['choices']
    ) == profile_ids
    for command in ('doctor', 'run'):
        rendered = _help(command)
        for profile_id, description in profile_help:
            assert f'{profile_id}: {description}' in rendered
    setup_profiles = tuple(
        contract['value_options']['setup']['--profile']['choices']
    )
    assert setup_profiles == profile_ids
    for command in ('start', 'setup'):
        assert tuple(
            contract['value_options'][command]['--profile']['choices']
        ) == setup_profiles
        rendered = _help(command)
        for profile_id, description in profile_help:
            assert f'{profile_id}: {description}' in rendered


def test_positionals_and_deprecation_lifecycle_are_machine_readable():
    """Directory requirements and the warning/removal policy are explicit."""
    contract = _contract()
    policy = contract['deprecation_policy']

    assert policy == {
        'minimum_compatibility_window': 'one minor release',
        'removal_status': 'not_scheduled',
        'warning_channel': 'stderr',
        'warning_count_per_invocation': 1,
    }
    for command in contract['commands'].values():
        positional = command['positional']
        assert positional['kind'] in {'directory', 'file'}
        assert positional['stability'] == 'stable'
        if 'must_contain' in positional:
            assert positional['must_contain']
            assert all(positional['must_contain'])


@pytest.mark.parametrize(
    ('arguments', 'message'),
    [
        (['unknown'], 'unknown command'),
        (['doctor', '/tmp', '--unknown'], 'unrecognized arguments'),
        (
            ['run', '/tmp', '--min-free-space-gib', '0'],
            'finite number greater than zero',
        ),
        (
            ['run', '/tmp', '--auto-exit-secs', '0'],
            'positive integer',
        ),
        (
            ['run', '/tmp', '--verification', 'maybe'],
            'invalid choice',
        ),
        (
            ['inspect', '/tmp', '--symptom', 'guess-the-fix'],
            'invalid choice',
        ),
        (
            [
                'run',
                '/tmp',
                '--verification',
                'off',
                '--no-verify-map',
            ],
            'cannot be combined',
        ),
        (
            ['run', '/tmp', '--resume', '--dry-run'],
            'cannot be combined',
        ),
        (
            ['run', '/tmp', '--yes'],
            'requires --guided',
        ),
        (
            ['start', '/tmp', '--json'],
            '--json requires --dry-run',
        ),
        (
            ['start', '/tmp', '--min-free-space-gib', '0'],
            'finite and greater than zero',
        ),
        (
            ['sessions', '/tmp', '--limit', '0'],
            'must be between 1 and 200',
        ),
        (
            [
                'run',
                '/tmp',
                '--guided',
                '--resume',
                '--output-dir',
                '/tmp/output',
            ],
            'cannot be combined with --resume',
        ),
        (
            ['run', '--help-all', '--dry-run'],
            'cannot be combined',
        ),
    ],
)
def test_invalid_options_have_stable_usage_exit(
    arguments: list[str],
    message: str,
):
    """Invalid names, values, and combinations should consistently exit 2."""
    completed = subprocess.run(
        [str(CLI), *arguments],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert message in completed.stderr
