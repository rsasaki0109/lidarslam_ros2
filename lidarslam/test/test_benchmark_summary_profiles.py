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

"""Regression tests for release-profile gate evaluation in benchmark_summary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import textwrap

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'benchmark_summary.py'
DEFAULT_PROFILE_YAML = REPO_ROOT / 'scripts' / 'release_profiles.yaml'


def _load_module():
    spec = importlib.util.spec_from_file_location('benchmark_summary', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rec(**overrides):
    base = {
        'run': overrides.get('run', 'r0'),
        'bag': overrides.get('bag', 'demo_bag'),
        'points_topic': overrides.get('points_topic', '/livox/lidar'),
        'ape_ref_kind': overrides.get('ape_ref_kind', 'ground_truth'),
        'ape_ref_src': overrides.get('ape_ref_src', 'leica_prism_gt'),
        'ape_rmse_m': overrides.get('ape_rmse_m', '0.500'),
        'ape_pairs': overrides.get('ape_pairs', 500),
        'provenance_complete': overrides.get('provenance_complete', True),
        'provenance_git_dirty': overrides.get('provenance_git_dirty', False),
        'provenance_git_commit': overrides.get(
            'provenance_git_commit', 'a' * 40),
    }
    return base


def test_default_release_profiles_yaml_loads():
    """The shipped scripts/release_profiles.yaml must parse without errors."""
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    names = [p['name'] for p in profiles]
    assert 'newer_college_math_hard' in names
    assert 'mid360_vs_glim' in names


def test_release_profiles_preserve_current_blocking_policy():
    """Blocking profiles stay blocking while the replaced cross-check warns."""
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    by_name = {p['name']: p for p in profiles}
    graduated = (
        'ntu_viral_tnp_01',
        'leo_drive_applanix_velodyne_cross',
    )
    for name in graduated:
        assert name in by_name, name
        assert by_name[name].get('report_only_until') is None, (
            f'{name} must remain a blocking release profile')
    assert by_name['mid360_vs_glim']['report_only_until'] == (
        'superseded-by-mid360-gt (D-GT-2)'
    )


def test_superseded_cross_validation_profile_warns_on_regression():
    """The superseded GLIM cross-check must not block the real-GT gate."""
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    mid360 = next(p for p in profiles if p['name'] == 'mid360_vs_glim')
    records = [
        _rec(run='regressed', points_topic='/livox/lidar',
             ape_ref_kind='cross_validation', ape_ref_src='glim_mid360_reference',
             ape_rmse_m='5.00', ape_pairs=600),
    ]
    [result] = module.evaluate_release_profiles([mid360], records)
    assert result['status'] == 'WARN'


def test_load_release_profiles_rejects_unknown_metric(tmp_path: Path):
    module = _load_module()
    bad = tmp_path / 'bad.yaml'
    bad.write_text(
        textwrap.dedent(
            """
            release_profiles:
              - name: x
                metric: not_a_metric
                pass: 1.0
            """
        )
    )
    with pytest.raises(ValueError, match='metric must be one of'):
        module.load_release_profiles(bad)


def test_load_release_profiles_rejects_duplicate_name(tmp_path: Path):
    module = _load_module()
    bad = tmp_path / 'bad.yaml'
    bad.write_text(
        textwrap.dedent(
            """
            release_profiles:
              - name: dup
                metric: ape_rmse_gt_m
                pass: 1.0
              - name: dup
                metric: ape_rmse_gt_m
                pass: 1.0
            """
        )
    )
    with pytest.raises(ValueError, match='duplicate profile name'):
        module.load_release_profiles(bad)


def test_load_release_profiles_rejects_non_boolean_provenance_gate(tmp_path: Path):
    module = _load_module()
    bad = tmp_path / 'bad.yaml'
    bad.write_text(
        textwrap.dedent(
            """
            release_profiles:
              - name: bad_provenance
                metric: ape_rmse_gt_m
                pass: 1.0
                match:
                  require_clean_provenance: "yes"
            """
        )
    )
    with pytest.raises(ValueError, match='require_clean_provenance must be boolean'):
        module.load_release_profiles(bad)


def test_load_release_profiles_rejects_empty_remediation(tmp_path: Path):
    """Remediation text must be actionable when a profile declares it."""
    module = _load_module()
    bad = tmp_path / 'bad_remediation.yaml'
    bad.write_text(
        '\n'.join([
            'release_profiles:',
            '  - name: bad_remediation',
            '    metric: ape_rmse_gt_m',
            '    pass: 0.1',
            '    remediation: ""',
            '    match: {}',
            '',
        ]),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='remediation must be a non-empty string'):
        module.load_release_profiles(bad)


def test_cli_rejects_malformed_required_git_commit(tmp_path: Path):
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            '--root',
            str(tmp_path),
            '--release-profile',
            str(DEFAULT_PROFILE_YAML),
            '--required-git-commit',
            'abc123',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    assert 'exactly 40 lowercase hexadecimal characters' in result.stdout


def test_cli_hard_profile_gate_requires_commit_binding(tmp_path: Path):
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            '--root',
            str(tmp_path),
            '--release-profile',
            str(DEFAULT_PROFILE_YAML),
            '--fail-on-profiles',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    assert 'requires --required-git-commit' in result.stdout


def test_cli_empty_root_renders_profile_blockers_and_remediation(tmp_path: Path):
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            '--root',
            str(tmp_path),
            '--release-profile',
            str(DEFAULT_PROFILE_YAML),
            '--fail-on-profiles',
            '--required-git-commit',
            'a' * 40,
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert '| newer_college_math_hard |' in result.stdout
    assert '| NO_DATA |' in result.stdout
    assert 'release profile gate FAILED' in result.stdout
    assert 'Follow docs/benchmarking.md#newer-college-maths-hard' in result.stdout


def test_cli_empty_root_without_profiles_preserves_missing_data_error(
    tmp_path: Path,
):
    result = subprocess.run(
        ['python3', str(SCRIPT_PATH), '--root', str(tmp_path)],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    assert f'no metrics.json found under: {tmp_path}' in result.stdout


def test_release_wrapper_empty_root_retains_actionable_profile_report(
    tmp_path: Path,
):
    benchmark_root = tmp_path / 'empty-benchmark-root'
    benchmark_root.mkdir()
    output_dir = tmp_path / 'release-readiness'

    result = subprocess.run(
        [
            'bash',
            str(REPO_ROOT / 'scripts' / 'run_release_readiness_checks.sh'),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(output_dir),
            '--fail-on-profiles',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert 'evaluating every release profile as missing evidence' in result.stdout
    summary = (output_dir / 'benchmark_summary.md').read_text(encoding='utf-8')
    log = (output_dir / 'benchmark_summary.log').read_text(encoding='utf-8')
    assert '| newer_college_math_hard |' in summary
    assert '| NO_DATA |' in summary
    assert 'Follow docs/benchmarking.md#newer-college-maths-hard' in log
    assert 'skipping benchmark HTML report' in (
        output_dir / 'benchmark_report.log'
    ).read_text(encoding='utf-8')


def test_evaluate_pass_picks_best_matching_run():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'bag_name_contains': 'tnp_01', 'reference_kind': 'ground_truth'},
    }
    records = [
        _rec(run='a', bag='tnp_01_run', ape_rmse_m='0.95'),
        _rec(run='b', bag='tnp_01_run', ape_rmse_m='0.80'),
        _rec(run='c', bag='unrelated_run', ape_rmse_m='0.10'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'PASS'
    assert result['best_run'] == 'b'
    assert result['best_value'] == pytest.approx(0.80)


def test_evaluate_target_met_when_under_target():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'target': 0.5,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.30')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'TARGET_MET'


def test_evaluate_fail_when_over_pass_without_report_only():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.95')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'FAIL'


def test_evaluate_warn_when_over_pass_with_report_only_until():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'report_only_until': 'v0.4',
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.95')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'WARN'


def test_evaluate_no_data_when_no_matches():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'match': {'bag_name_contains': 'nothing'},
    }
    records = [_rec(run='a', bag='foo', ape_rmse_m='0.5')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'NO_DATA'
    assert result['best_run'] is None


@pytest.mark.parametrize(
    ('complete', 'dirty'),
    [(False, False), (True, True), (False, None)],
)
def test_clean_provenance_match_fails_closed(complete, dirty):
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'require_clean_provenance': True},
    }
    records = [
        _rec(
            ape_rmse_m='0.01',
            provenance_complete=complete,
            provenance_git_dirty=dirty,
        ),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'NO_DATA'
    assert result['candidate_runs'] == 1
    if complete and dirty:
        assert result['no_data_reason'] == 'dirty revision: r0'
    else:
        assert result['no_data_reason'] == 'incomplete provenance: r0'


def test_provenance_rejection_diagnostics_preserve_multiple_causes():
    """NO_DATA should identify incomplete and dirty candidate runs."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'require_clean_provenance': True},
    }
    records = [
        _rec(run='legacy', provenance_complete=False),
        _rec(run='local', provenance_git_dirty=True),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    rendered = '\n'.join(module.render_release_profile_section([result]))

    assert result['status'] == 'NO_DATA'
    assert result['candidate_runs'] == 2
    assert result['no_data_reason'] == (
        'incomplete provenance: legacy; dirty revision: local'
    )
    assert result['no_data_reason'] in rendered


def test_blocking_no_data_renders_actionable_remediation():
    """A blocking profile must carry its recovery path into the report."""
    module = _load_module()
    profile = {
        'name': 'form_gated_dataset',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.1,
        'remediation': 'Follow docs/benchmarking.md#dataset and rerun.',
        'match': {'bag_name_contains': 'missing'},
    }
    [result] = module.evaluate_release_profiles([profile], [])
    rendered = '\n'.join(module.render_release_profile_section([result]))

    assert result['status'] == 'NO_DATA'
    assert result['remediation'] == profile['remediation']
    assert '### Blocking profile remediation' in rendered
    assert '`form_gated_dataset`' in rendered
    assert profile['remediation'] in rendered


def test_blocking_profile_requires_exact_release_candidate_commit():
    """Clean evidence from an older revision must fail the release gate."""
    module = _load_module()
    candidate_commit = 'b' * 40
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'require_clean_provenance': True},
    }
    records = [
        _rec(
            run='old_clean_run',
            ape_rmse_m='0.01',
            provenance_git_commit='a' * 40,
        ),
    ]

    [result] = module.evaluate_release_profiles(
        [profile],
        records,
        required_git_commit=candidate_commit,
    )
    rendered = '\n'.join(module.render_release_profile_section([result]))

    assert result['status'] == 'NO_DATA'
    assert result['matched_runs'] == 0
    assert result['no_data_reason'] == (
        'candidate commit mismatch (required bbbbbbbbbbbb): '
        'old_clean_run@aaaaaaaaaaaa'
    )
    assert result['no_data_reason'] in rendered


def test_blocking_profile_accepts_exact_release_candidate_commit():
    """Clean evidence from the requested revision remains eligible."""
    module = _load_module()
    candidate_commit = 'b' * 40
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'require_clean_provenance': True},
    }
    records = [
        _rec(
            run='candidate_run',
            ape_rmse_m='0.20',
            provenance_git_commit=candidate_commit,
        ),
    ]

    [result] = module.evaluate_release_profiles(
        [profile],
        records,
        required_git_commit=candidate_commit,
    )
    rendered = '\n'.join(module.render_release_profile_section([result]))

    assert result['status'] == 'PASS'
    assert result['best_run'] == 'candidate_run'
    assert 'clean provenance @ bbbbbbbbbbbb' in rendered


def test_report_only_profile_remains_a_cross_revision_comparison():
    """Commit binding applies only to profiles that can block release."""
    module = _load_module()
    profile = {
        'name': 'historical',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'report_only_until': 'future-cycle',
        'match': {'require_clean_provenance': True},
    }
    records = [
        _rec(
            run='older_baseline',
            provenance_git_commit='a' * 40,
        ),
    ]

    [result] = module.evaluate_release_profiles(
        [profile],
        records,
        required_git_commit='b' * 40,
    )

    assert result['status'] == 'PASS'
    assert result['best_run'] == 'older_baseline'
    assert result['required_git_commit'] is None


def test_metric_ape_rmse_gt_m_skips_cross_validation():
    """ape_rmse_gt_m must only score ground_truth references."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [
        _rec(run='cv', bag='foo', ape_ref_kind='cross_validation', ape_rmse_m='0.10'),
        _rec(run='gt', bag='foo', ape_ref_kind='ground_truth', ape_rmse_m='0.50'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'gt'
    assert result['best_value'] == pytest.approx(0.50)


def test_min_ape_pairs_filters_incomplete_runs():
    """Match should drop runs whose APE was computed on too few pose pairs."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_vs_reference_m',
        'pass': 4.0,
        'match': {
            'points_topic': '/livox/lidar',
            'reference_kind': 'cross_validation',
            'min_ape_pairs': 400,
        },
    }
    records = [
        _rec(run='partial', ape_ref_kind='cross_validation', ape_rmse_m='0.11', ape_pairs=119),
        _rec(run='full', ape_ref_kind='cross_validation', ape_rmse_m='3.45', ape_pairs=580),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'full'
    assert result['best_value'] == pytest.approx(3.45)


def test_profile_match_combines_predicates():
    """All predicates in match must hold simultaneously (AND semantics)."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_vs_reference_m',
        'pass': 1.0,
        'match': {
            'points_topic': '/livox/lidar',
            'reference_source_contains': 'glim_mid360',
        },
    }
    records = [
        _rec(run='other_topic',
             points_topic='/os1/points',
             ape_ref_kind='cross_validation',
             ape_ref_src='glim_mid360_reference',
             ape_rmse_m='0.1'),
        _rec(run='other_ref',
             points_topic='/livox/lidar',
             ape_ref_kind='cross_validation',
             ape_ref_src='applanix_gsof49_reference',
             ape_rmse_m='0.2'),
        _rec(run='match_me',
             points_topic='/livox/lidar',
             ape_ref_kind='cross_validation',
             ape_ref_src='glim_mid360_reference',
             ape_rmse_m='0.3'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'match_me'


def test_render_release_profile_section_has_required_columns():
    module = _load_module()
    results = [
        {
            'name': 'newer',
            'status': 'PASS',
            'metric': 'ape_rmse_gt_m',
            'best_run': 'r1',
            'best_value': 0.08,
            'pass': 0.10,
            'target': 0.08,
            'report_only_until': None,
        }
    ]
    lines = module.render_release_profile_section(results)
    body = '\n'.join(lines)
    assert '## Release profile gate' in body
    assert 'newer' in body
    assert 'PASS' in body
    assert 'ape_rmse_gt_m' in body
