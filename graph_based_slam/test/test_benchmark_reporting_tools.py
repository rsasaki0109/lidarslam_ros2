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

"""Regression tests for benchmark summary and release-readiness scripts."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_SUMMARY_SCRIPT = REPO_ROOT / 'scripts' / 'benchmark_summary.py'
FIXTURE_SCRIPT = REPO_ROOT / 'scripts' / 'generate_sample_benchmark_metrics.py'
RELEASE_READINESS_SCRIPT = (
    REPO_ROOT / 'scripts' / 'run_release_readiness_checks.sh'
)


def _write_json(path: Path, payload: dict) -> None:
    """Write a JSON fixture."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def _write_metrics(
    run_dir: Path,
    *,
    bag_name: str,
    ape_rmse: float | None,
    lid_ok: bool = True,
    lid_rtf: float = 0.5,
    lid_wall: float = 12.0,
    glim_ok: bool = True,
    glim_rtf: float = 0.8,
    glim_wall: float = 20.0,
) -> None:
    """Write a minimal metrics.json fixture for report scripts."""
    run_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        'out_dir': str(run_dir),
        'bag_path': f'/bags/{bag_name}',
        'bag_duration_sec': 30.0,
        'points_topic': '/points',
        'frames': {'points_frame_id': 'os_sensor'},
        'reference': {
            'source': 'leica_prism_gt',
            'kind': 'ground_truth',
        },
        'lidarslam': {
            'success': lid_ok,
            'rtf': lid_rtf,
            'wall_sec': lid_wall,
            'param_path': str(REPO_ROOT / 'lidarslam/param/lidarslam.yaml'),
        },
        'glim': {
            'available': True,
            'success': glim_ok,
            'reference_source': 'cache',
            'rtf': glim_rtf,
            'wall_sec': glim_wall,
        },
    }
    if ape_rmse is not None:
        metrics['evo'] = {
            'ape': {
                'rmse': ape_rmse,
                'median': ape_rmse / 2.0,
                'max': ape_rmse * 2.0,
            }
        }

    (run_dir / 'metrics.json').write_text(
        json.dumps(metrics, indent=2),
        encoding='utf-8',
    )


def _write_minimal_autoware_map(run_dir: Path, *, tum_poses: int) -> None:
    """Write a minimal map directory accepted by the Autoware map verifier."""
    pointcloud_map = run_dir / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True, exist_ok=True)
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: local\n',
        encoding='utf-8',
    )
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20\n'
        'y_resolution: 20\n'
        '0_0.pcd: [0, 0]\n',
        encoding='utf-8',
    )
    (pointcloud_map / '0_0.pcd').write_text(
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH 1\n'
        'HEIGHT 1\n'
        'POINTS 1\n'
        'DATA ascii\n'
        '1.0 2.0 0.0\n',
        encoding='ascii',
    )
    (run_dir / 'fake_tum_0.txt').write_text(
        ''.join(f'{float(index):.1f} {index} 0 0 0 0 0 1\n' for index in range(tum_poses)),
        encoding='ascii',
    )


def _write_public_mid360_completion_fixture(
    root: Path,
    *,
    matched_case: str = 'case_a',
    recommended_case: str = 'case_a',
) -> dict[str, Path]:
    """Write artifacts consumed by the public MID-360 completion gate."""
    start_run = root / 'segment_000'
    end_run = root / 'segment_012'
    _write_minimal_autoware_map(start_run, tum_poses=60)
    _write_minimal_autoware_map(end_run, tum_poses=70)
    dashboard = root / 'dashboard.html'
    dashboard.write_text('<html>dashboard</html>\n', encoding='utf-8')
    loop_cloud = root / 'loop_cloud.json'
    segment_plan = root / 'segment_plan.json'
    alignment = root / 'alignment.json'
    adoption = root / 'adoption.json'
    _write_json(loop_cloud, {'status': 'PASS', 'overlap': {'symmetric_median_nn_m': 0.2}})
    _write_json(
        segment_plan,
        {
            'status': 'PASS',
            'reset_pair': {
                'start': {'status': 'PASS', 'segment': {'segment_id': 'segment_000'}},
                'end': {'status': 'PASS', 'segment': {'segment_id': 'segment_012'}},
            },
        },
    )
    _write_json(
        alignment,
        {
            'status': 'PASS',
            'aligned_overlap': {
                'symmetric_median_nn_m': 0.6,
                'symmetric_p90_nn_m': 2.1,
            },
        },
    )
    _write_json(
        adoption,
        {
            'status': 'PASS',
            'decision': {
                'matched_case': matched_case,
                'recommended_case': recommended_case,
                'gate_pass_cases': 2,
            },
        },
    )
    return {
        'loop_cloud': loop_cloud,
        'segment_plan': segment_plan,
        'start_run': start_run,
        'end_run': end_run,
        'alignment': alignment,
        'adoption': adoption,
        'dashboard': dashboard,
    }


def test_benchmark_summary_ranks_runs_and_writes_artifacts(tmp_path):
    """Benchmark summary should rank by APE and emit CSV/Markdown."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_slower',
        bag_name='run_slower',
        ape_rmse=0.25,
        lid_rtf=0.62,
    )
    _write_metrics(
        root / 'suite_a' / 'run_best',
        bag_name='run_best',
        ape_rmse=0.09,
        lid_rtf=0.57,
    )
    _write_metrics(
        root / 'suite_b' / 'run_no_ape',
        bag_name='run_no_ape',
        ape_rmse=None,
        lid_rtf=0.41,
    )

    markdown_path = tmp_path / 'summary.md'
    csv_path = tmp_path / 'summary.csv'
    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--write-md',
            str(markdown_path),
            '--write-csv',
            str(csv_path),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert '- runs: 3' in result.stdout
    assert 'APE-primary best: run_best (0.090m)' in result.stdout
    assert markdown_path.is_file()
    assert csv_path.is_file()

    with csv_path.open('r', encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))

    assert [row['run'] for row in rows] == [
        'run_best',
        'run_slower',
        'run_no_ape',
    ]
    assert rows[0]['primary_rank'] == '1'
    assert rows[0]['ape_ok'] == 'true'
    assert rows[0]['ape_ref_kind'] == 'ground_truth'
    assert rows[0]['ape_ref_src'] == 'leica_prism_gt'
    assert rows[1]['ape_ok'] == 'false'
    assert rows[2]['ape_rmse_m'] == ''


def test_benchmark_summary_fails_when_metrics_are_missing(tmp_path):
    """Empty benchmark roots should fail with a clear message."""
    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    assert 'no metrics.json found under' in result.stdout


def test_benchmark_summary_can_fail_on_threshold(tmp_path):
    """Threshold gate should fail when a run exceeds the requested APE."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_good',
        bag_name='run_good',
        ape_rmse=0.08,
    )
    _write_metrics(
        root / 'suite_a' / 'run_bad',
        bag_name='run_bad',
        ape_rmse=0.22,
    )

    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--ape-threshold',
            '0.10',
            '--fail-on-ape-threshold',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert 'error: APE threshold failed for runs: run_bad' in result.stdout


def test_benchmark_summary_threshold_can_filter_reference_kind(tmp_path):
    """Cross-validation runs should not fail a ground-truth-only threshold gate."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_gt',
        bag_name='run_gt',
        ape_rmse=0.08,
    )
    crossval_dir = root / 'suite_a' / 'run_crossval'
    _write_metrics(
        crossval_dir,
        bag_name='run_crossval',
        ape_rmse=0.22,
    )
    metrics = json.loads((crossval_dir / 'metrics.json').read_text(encoding='utf-8'))
    metrics['reference'] = {
        'source': 'glim_mid360_reference',
        'kind': 'cross_validation',
    }
    (crossval_dir / 'metrics.json').write_text(
        json.dumps(metrics, indent=2),
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--ape-threshold',
            '0.10',
            '--ape-threshold-reference-kind',
            'ground_truth',
            '--fail-on-ape-threshold',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert 'reference_kind=ground_truth' in result.stdout


def _run_release_readiness(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(RELEASE_READINESS_SCRIPT), *args],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )


def test_release_readiness_help_exits_successfully():
    result = _run_release_readiness('--help')

    assert result.returncode == 0
    assert 'run_release_readiness_checks.sh' in result.stderr
    assert '--skip-default-ci' in result.stderr


def test_release_readiness_rejects_missing_option_value_before_realpath():
    result = _run_release_readiness('--out-dir', '--skip-default-ci')

    assert result.returncode == 2
    assert 'error: option requires a value: --out-dir' in result.stderr
    assert 'realpath' not in result.stderr


def test_release_readiness_rejects_unknown_option_with_help_hint():
    result = _run_release_readiness('--bogus')

    assert result.returncode == 2
    assert 'error: unknown option: --bogus' in result.stderr
    assert 'run_release_readiness_checks.sh --help' in result.stderr
    assert 'Release readiness output:' not in result.stdout


def test_release_readiness_can_skip_expensive_stages(tmp_path):
    out_dir = tmp_path / 'release_readiness_cli'

    result = _run_release_readiness(
        '--skip-default-ci',
        '--skip-benchmark-summary',
        '--out-dir',
        str(out_dir),
    )

    assert result.returncode == 0, result.stderr
    assert out_dir.is_dir()
    assert 'Release readiness checks completed' in result.stdout


def test_release_readiness_fails_closed_without_threshold_evidence(tmp_path):
    """A requested APE gate must not pass with an empty benchmark root."""
    benchmark_root = tmp_path / 'empty'
    benchmark_root.mkdir()
    out_dir = tmp_path / 'release_readiness'

    result = _run_release_readiness(
        '--skip-default-ci',
        '--benchmark-root',
        str(benchmark_root),
        '--out-dir',
        str(out_dir),
        '--ape-threshold',
        '0.10',
    )

    assert result.returncode == 2
    assert 'requested benchmark gate has no evidence' in result.stderr
    assert not (out_dir / 'benchmark_report.html').exists()


def test_release_readiness_allows_empty_report_only_benchmark_root(tmp_path):
    """No metrics may be skipped only when no benchmark hard gate is active."""
    benchmark_root = tmp_path / 'empty'
    benchmark_root.mkdir()
    out_dir = tmp_path / 'release_readiness'

    result = _run_release_readiness(
        '--skip-default-ci',
        '--benchmark-root',
        str(benchmark_root),
        '--out-dir',
        str(out_dir),
    )

    assert result.returncode == 0, result.stderr
    assert 'benchmark reporting is skipped' in result.stdout


def test_release_readiness_rejects_skipping_a_requested_gate(tmp_path):
    """An option conflict must not silently disable the APE gate."""
    result = _run_release_readiness(
        '--skip-default-ci',
        '--skip-benchmark-summary',
        '--ape-threshold',
        '0.10',
        '--out-dir',
        str(tmp_path / 'out'),
    )

    assert result.returncode == 2
    assert 'cannot disable a requested benchmark gate' in result.stderr


def test_release_readiness_requires_profile_for_hard_profile_gate(tmp_path):
    """A hard profile gate must identify an existing profile."""
    no_profile = _run_release_readiness(
        '--skip-default-ci',
        '--no-release-profile',
        '--fail-on-profiles',
        '--out-dir',
        str(tmp_path / 'no_profile'),
    )
    missing_profile = _run_release_readiness(
        '--skip-default-ci',
        '--release-profile',
        str(tmp_path / 'missing.yaml'),
        '--fail-on-profiles',
        '--out-dir',
        str(tmp_path / 'missing_profile'),
    )

    assert no_profile.returncode == 2
    assert 'requires an active --release-profile' in no_profile.stderr
    assert missing_profile.returncode == 2
    assert 'release profile not found' in missing_profile.stderr


def test_synthetic_fixture_generator_drives_release_gate(tmp_path):
    """The synthetic fixture should exercise the uniform-threshold plumbing."""
    benchmark_root = tmp_path / 'fixture'
    out_dir = tmp_path / 'release_ready'

    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'passing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert fixture.returncode == 0, fixture.stderr
    assert (benchmark_root / 'ci_fixture' / 'run_best' / 'metrics.json').is_file()
    assert (benchmark_root / 'references' / 'synthetic_reference.tum').is_file()

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert (out_dir / 'benchmark_report.html').is_file()
    assert 'run_best' in (out_dir / 'benchmark_summary.md').read_text(
        encoding='utf-8',
    )


def test_synthetic_fixture_cannot_satisfy_release_profiles(tmp_path):
    """Unrelated passing metrics must not satisfy blocking dataset profiles."""
    benchmark_root = tmp_path / 'fixture'
    out_dir = tmp_path / 'release_profiles'

    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'passing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )
    assert fixture.returncode == 0, fixture.stderr

    result = _run_release_readiness(
        '--skip-default-ci',
        '--benchmark-root',
        str(benchmark_root),
        '--out-dir',
        str(out_dir),
        '--fail-on-profiles',
    )

    assert result.returncode == 2
    assert 'newer_college_math_hard (NO_DATA)' in result.stdout
    assert 'ntu_viral_tnp_01 (NO_DATA)' in result.stdout
    assert 'hint: newer_college_math_hard:' in result.stdout
    assert 'docs/benchmarking.md#newer-college-maths-hard' in result.stdout
    assert not (out_dir / 'benchmark_report.html').exists()


def test_report_only_profile_may_remain_without_data(tmp_path):
    """Missing data stays non-blocking only for an explicit report-only row."""
    benchmark_root = tmp_path / 'fixture'
    profile_path = tmp_path / 'report_only.yaml'
    out_dir = tmp_path / 'report_only'
    profile_path.write_text(
        '\n'.join([
            'release_profiles:',
            '  - name: optional_dataset',
            '    metric: ape_rmse_gt_m',
            '    pass: 0.10',
            '    report_only_until: future-cycle',
            '    match:',
            '      bag_name_contains: absent_dataset',
            '',
        ]),
        encoding='utf-8',
    )
    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'passing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )
    assert fixture.returncode == 0, fixture.stderr

    result = _run_release_readiness(
        '--skip-default-ci',
        '--benchmark-root',
        str(benchmark_root),
        '--out-dir',
        str(out_dir),
        '--release-profile',
        str(profile_path),
        '--fail-on-profiles',
    )

    assert result.returncode == 0, result.stderr
    expected_commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        capture_output=True,
        text=True,
        check=True,
        cwd=REPO_ROOT,
    ).stdout.strip()
    assert f'Release candidate commit: {expected_commit}' in result.stdout
    assert '| optional_dataset | NO_DATA |' in result.stdout
    assert (out_dir / 'benchmark_report.html').is_file()


def test_synthetic_fixture_generator_failing_profile_trips_release_gate(tmp_path):
    """The failing fixture profile should trip the release gate."""
    benchmark_root = tmp_path / 'fixture_fail'
    out_dir = tmp_path / 'release_fail'

    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'failing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert fixture.returncode == 0, fixture.stderr
    assert (benchmark_root / 'ci_fixture' / 'run_bad' / 'metrics.json').is_file()

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert not (out_dir / 'benchmark_report.html').exists()
    assert 'run_bad' in (out_dir / 'benchmark_summary.md').read_text(
        encoding='utf-8',
    )


def test_release_readiness_generates_summary_and_html_report(tmp_path):
    """Release-readiness wrapper should emit benchmark artifacts."""
    benchmark_root = tmp_path / 'benchmarks'
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_good',
        bag_name='math_hard',
        ape_rmse=0.08,
    )
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_ok_too',
        bag_name='math_hard',
        ape_rmse=0.09,
    )
    out_dir = tmp_path / 'release_readiness'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert (out_dir / 'benchmark_summary.csv').is_file()
    assert (out_dir / 'benchmark_report.html').is_file()
    assert 'benchmark_summary_md:' in result.stdout
    assert 'benchmark_report_html:' in result.stdout
    html = (out_dir / 'benchmark_report.html').read_text(
        encoding='utf-8',
    )
    assert 'run_good' in html
    assert 'newer_college' in html


def test_release_readiness_fails_on_threshold_violation(tmp_path):
    """Release-readiness should stop when the APE gate is violated."""
    benchmark_root = tmp_path / 'benchmarks'
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_good',
        bag_name='math_hard',
        ape_rmse=0.08,
    )
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_bad',
        bag_name='math_hard',
        ape_rmse=0.22,
    )
    out_dir = tmp_path / 'release_readiness_fail'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert not (out_dir / 'benchmark_report.html').exists()
    assert 'error: APE threshold failed for runs: run_bad' in result.stdout


def test_release_readiness_can_run_public_mid360_completion_gate(tmp_path):
    """Release-readiness should optionally hard-gate public MID-360 completion."""
    fixture = _write_public_mid360_completion_fixture(tmp_path / 'public_mid360_fixture')
    out_dir = tmp_path / 'release_readiness'
    gate_dir = out_dir / 'public_mid360_gate'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--skip-benchmark-summary',
            '--out-dir',
            str(out_dir),
            '--public-mid360-completion',
            '--public-mid360-completion-output-dir',
            str(gate_dir),
            '--public-mid360-loop-cloud',
            str(fixture['loop_cloud']),
            '--public-mid360-segment-reset-plan',
            str(fixture['segment_plan']),
            '--public-mid360-start-run-dir',
            str(fixture['start_run']),
            '--public-mid360-end-run-dir',
            str(fixture['end_run']),
            '--public-mid360-segment-map-alignment',
            str(fixture['alignment']),
            '--public-mid360-adoption-gate',
            str(fixture['adoption']),
            '--public-mid360-dashboard-html',
            str(fixture['dashboard']),
            '--public-mid360-min-segment-rko-poses',
            '50',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report_path = gate_dir / 'mid360_robot_public_completion_gate.json'
    assert report_path.is_file()
    report = json.loads(report_path.read_text(encoding='utf-8'))
    assert report['status'] == 'PASS'
    assert report['counts']['pass'] == 11
    assert 'public_mid360_completion_gate_json:' in result.stdout


def test_release_readiness_fails_when_public_mid360_completion_gate_fails(tmp_path):
    """The public MID-360 completion hook should be a hard gate when enabled."""
    fixture = _write_public_mid360_completion_fixture(
        tmp_path / 'public_mid360_fixture',
        matched_case='case_b',
        recommended_case='case_a',
    )
    out_dir = tmp_path / 'release_readiness'
    gate_dir = out_dir / 'public_mid360_gate'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--skip-benchmark-summary',
            '--out-dir',
            str(out_dir),
            '--public-mid360-completion',
            '--public-mid360-completion-output-dir',
            str(gate_dir),
            '--public-mid360-loop-cloud',
            str(fixture['loop_cloud']),
            '--public-mid360-segment-reset-plan',
            str(fixture['segment_plan']),
            '--public-mid360-start-run-dir',
            str(fixture['start_run']),
            '--public-mid360-end-run-dir',
            str(fixture['end_run']),
            '--public-mid360-segment-map-alignment',
            str(fixture['alignment']),
            '--public-mid360-adoption-gate',
            str(fixture['adoption']),
            '--public-mid360-dashboard-html',
            str(fixture['dashboard']),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    report = json.loads(
        (gate_dir / 'mid360_robot_public_completion_gate.json').read_text(encoding='utf-8')
    )
    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'tracked_config_matches_top_gate' and check['status'] == 'FAIL'
        for check in report['checks']
    )
