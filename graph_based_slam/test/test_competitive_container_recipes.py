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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Static contract tests for the pinned competitive benchmark recipes."""

import hashlib
import importlib.util
import json
from pathlib import Path
import re

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
SPEC = importlib.util.spec_from_file_location(
    'run_competitive_gt_blind_benchmark',
    ROOT / 'scripts' / 'run_competitive_gt_blind_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)
RECIPE_NAMES = {
    'ours': 'docker/ours_competitive_benchmark.Dockerfile',
    'glim': 'docker/glim_cpu_benchmark.Dockerfile',
    'fast_livo2': 'docker/fast_livo2_benchmark.Dockerfile',
}
THREAD_ENV_NAMES = (
    'OMP_NUM_THREADS',
    'OPENBLAS_NUM_THREADS',
    'MKL_NUM_THREADS',
    'TBB_NUM_THREADS',
)
OVERHEAD_PREREGISTRATION = ROOT / (
    'configs/slam_benchmark_profiles/'
    'm6a7_sampler_overhead_preregistration.json')
OVERHEAD_PREREGISTRATION_V2 = ROOT / (
    'configs/slam_benchmark_profiles/'
    'm6a7_sampler_overhead_preregistration_v2.json')
OVERHEAD_PREREGISTRATION_V3 = ROOT / (
    'configs/slam_benchmark_profiles/'
    'm6a7_sampler_overhead_preregistration_v3.json')


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _receipt():
    return yaml.safe_load(RECEIPT.read_text())


def test_all_owned_recipes_pin_base_and_cpu_contract():
    for recipe_name in RECIPE_NAMES.values():
        recipe = ROOT / recipe_name
        from_line = next(
            line for line in recipe.read_text().splitlines()
            if line.startswith('FROM '))
        assert re.fullmatch(r'FROM \S+@sha256:[0-9a-f]{64}', from_line)
        text = recipe.read_text()
        assert 'BENCHMARK_CPU_ONLY=1' in text
        for env_name in THREAD_ENV_NAMES:
            assert env_name in text


def test_receipt_binds_recipe_and_build_entrypoint_hashes():
    # The checked-in r1 receipt is retained as historical evidence and is
    # intentionally superseded by the current r2 selection.  Validate each
    # recipe against the current immutable selection, while preserving the
    # old receipt bytes for audit rather than pretending they are current.
    historical = _receipt()['systems']
    current = yaml.safe_load((ROOT / 'configs/slam_benchmark_profiles/'
                             'competitive_execution_selection_2026-08-r2.yaml').read_text())
    assert current['closure_id'] == 'competitive-rival-source-closure-2026-08-r2'
    assert current['closure_revision'] == 2
    for system, recipe_name in RECIPE_NAMES.items():
        if system == 'ours':
            recipe = historical[system]['container']['recipe']
            assert recipe['path'] == recipe_name
            assert recipe['sha256'] == _sha256(ROOT / recipe_name)
            continue
        recipe = current['current_recipe_bindings'].get(system)
        assert recipe['dockerfile']['path'] == recipe_name
        assert recipe['dockerfile']['sha256'] == _sha256(ROOT / recipe_name)
        assert recipe['build_script']['sha256'] == _sha256(
            ROOT / recipe['build_script']['path'])
        assert re.fullmatch(r'sha256:[0-9a-f]{64}',
                            historical[system]['container']['recipe']['base_digest'])
    assert historical['ours']['container']['build_entrypoint_sha256'] != (
        _sha256(ROOT / 'scripts/build_competitive_benchmark_images.sh'))
    ours_recipe = historical['ours']['container']['recipe']
    assert ours_recipe['source_repository'] == (
        'https://github.com/rsasaki0109/lidar_slam_ros2.git')
    assert ours_recipe['source_checkout'] == (
        'clone_at_revision_with_required_submodule_only')
    assert historical['glim']['toolchain']['not_applicable_fields'] == ['pcl']
    assert historical['glim']['toolchain']['status'] == 'ready'
    assert historical['glim']['toolchain']['scope'] == 'system_container'
    assert historical['glim']['toolchain']['observed']['pcl'] == 'not_applicable'
    assert historical['fast_livo2']['toolchain']['status'] == 'ready'
    assert historical['fast_livo2']['toolchain']['scope'] == 'system_container'
    assert historical['fast_livo2']['toolchain']['not_applicable_fields'] == []
    assert historical['ours']['toolchain']['status'] == 'ready'
    assert historical['ours']['toolchain']['scope'] == 'system_container'
    assert historical['ours']['toolchain']['not_applicable_fields'] == []


def test_fast_recipe_has_pinned_sources_and_no_local_base():
    text = (ROOT / RECIPE_NAMES['fast_livo2']).read_text()
    assert 'FAST_LIVO2_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64' in text
    assert 'RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd' in text
    assert 'SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c' in text
    assert 'FROM hdl_localization_noetic:local' not in text
    assert 'FROM fast-livo2-benchmark:noetic' not in text
    assert 'native_flags=disabled' in text


def test_glim_recipe_does_not_add_unused_pcl_to_toolchain():
    text = (ROOT / RECIPE_NAMES['glim']).read_text()
    assert 'libpcl-dev' not in text


def test_build_entrypoint_is_pull_free_and_revision_pinned():
    text = (ROOT / 'scripts/build_competitive_benchmark_images.sh').read_text()
    assert 'docker build --pull=false' in text
    assert 'OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git' in text
    assert 'archive --format=tar' in text
    assert 'RKO_LIO_ARCHIVE_SHA256=' in text
    assert 'OURS_PATCH_SHA256=' in text
    assert 'rko_lio.m6a10-v2a.patch' in text
    assert '--build-arg "RKO_LIO_PATCH_SHA256=$OURS_PATCH_SHA256"' in text
    assert 'cp "$ROOT/docker/ours_competitive_benchmark.Dockerfile"' in text
    assert '--build-arg "OURS_REPOSITORY=$OURS_REPOSITORY"' in text
    assert '--build-arg "OURS_REVISION=$OURS_REVISION"' in text
    assert 'lidarslam-ours:jazzy' in text
    assert 'glim-cpu-benchmark:competitive-v1' in text
    assert 'fast-livo2-benchmark:ros1-pinned' in text
    for revision in (
            '866f733677e92ecb08d67126e463da99dd140d46',
            'faa264a1bce1bda406f73457e35511f56cdc2eaa',
            '4a9e7a4cb084967c8525a1be529ad3ba2a118ae7',
            '0d2c0346107b75b59934975adec9a6eeeb913c64',
            '6c886c8e5d83997806e00294826d528cea3581dd',
            'a621ff2e56c56c839a6c40418d42c3c254424b5c',
    ):
        assert revision in text
    assert 'rosbag' not in text.lower()
    assert 'ground_truth' not in text.lower()


def test_ours_recipe_clones_revision_and_verifies_submodules_in_image():
    text = (ROOT / RECIPE_NAMES['ours']).read_text()
    assert 'ARG OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git' in text
    assert 'git clone --no-checkout "$OURS_REPOSITORY"' in text
    assert 'checkout --detach "$OURS_REVISION"' in text
    assert 'submodule sync --recursive' in text
    assert 'submodule update --init --recursive -- Thirdparty/ndt_omp_ros2' in text
    assert 'Thirdparty/rko_lio' in text
    assert 'NDT_OMP_SUBMODULE_REVISION=497411279593eb261a3e3d04cdcbb4717af33ca3' in text
    assert 'RKO_LIO_GITLINK_REVISION=622b74778a41f753d47aa5918043755ebcbd4c75' in text
    assert 'BUILD_TESTING=OFF' in text
    assert 'colcon list --base-paths' in text
    assert 'rev-parse HEAD' in text
    assert 'submodule status --recursive' in text
    assert "awk '$1 ~ /^[-+U]/" in text
    assert 'COPY rko_lio.tar /tmp/rko_lio.tar' in text
    assert 'RKO_LIO_ARCHIVE_SHA256=' in text
    assert 'sha256sum /tmp/rko_lio.tar' in text
    assert (
        'ARG RKO_LIO_PATCH_SHA256='
        '67a7b0f9c0118e51604fd89690f682b40f1c24fcd5b39ed216e04bc47e8ee503'
    ) in text
    assert 'COPY rko_lio.m6a10-v2a.patch /tmp/rko_lio.m6a10-v2a.patch' in text
    assert 'patch --batch --forward --strip=1 < /tmp/rko_lio.m6a10-v2a.patch' in text
    assert (
        'benchmark.rko_lio.patch_sha256="'
        '67a7b0f9c0118e51604fd89690f682b40f1c24fcd5b39ed216e04bc47e8ee503"'
    ) in text
    assert 'benchmark.rko_lio.initialized="true"' in text


def test_ours_recipe_explicitly_binds_benchmark_launch_overlay():
    recipe = (ROOT / RECIPE_NAMES['ours']).read_text()
    build = (ROOT / 'scripts/build_competitive_benchmark_images.sh').read_text()
    launch_path = ROOT / 'lidarslam' / 'launch' / 'rko_lio_slam.launch.py'
    launch_sha = _sha256(launch_path)

    assert f'ARG OURS_LAUNCH_SHA256={launch_sha}' in recipe
    assert 'COPY rko_lio_slam.launch.py /tmp/rko_lio_slam.launch.py' in recipe
    assert 'sha256sum /tmp/rko_lio_slam.launch.py' in recipe
    assert 'install -m 0644 /tmp/rko_lio_slam.launch.py' in recipe
    assert (
        f'benchmark.ours.launch_sha256="{launch_sha}"' in recipe)
    assert 'benchmark.ours.launch_path="lidarslam/launch/rko_lio_slam.launch.py"' in recipe

    assert 'cp "$ROOT/lidarslam/launch/rko_lio_slam.launch.py"' in build
    assert '"$OURS_CONTEXT/rko_lio_slam.launch.py"' in build
    assert 'OURS_LAUNCH_SHA256=' in build
    assert '--build-arg "OURS_LAUNCH_SHA256=$OURS_LAUNCH_SHA256"' in build
    assert 'docker build --pull=false' in build
    assert '"$OURS_CONTEXT" ;;' in build


def test_ours_runtime_contract_checks_rko_layout_and_entrypoint():
    text = (ROOT / 'scripts' / 'ours_container_gt_blind_run.sh').read_text()
    assert 'ros2 pkg prefix rko_lio' in text
    assert '${RKO_PREFIX}/share/rko_lio/package.xml' in text
    assert '${RKO_PREFIX}/lib/rko_lio/offline_node' in text
    assert 'rko_lio_hilti2022_pandar.yaml' in text


def test_synthetic_smoke_paths_are_full_startup_contracts():
    ours = (ROOT / 'scripts' / 'ours_container_gt_blind_run.sh').read_text()
    glim = (ROOT / 'scripts' / 'glim_container_run.sh').read_text()
    fast = (ROOT / 'scripts' / 'fast_livo2_container_run.sh').read_text()
    memory = (ROOT / 'scripts' / 'container_memory_evidence.sh').read_text()
    assert 'container_memory.json' in memory
    assert 'container_process_rss.json' in memory
    assert 'm6a7_start_process_rss_sampler' in memory
    assert 'm6a7_stop_process_rss_sampler' in memory
    assert 'm6a5_install_container_signal_traps' in memory
    assert 'm6a5_container_signal_trap' in memory
    assert 'M6A7_SAMPLER_STOP_TIMEOUT_SECS' in memory
    assert 'chmod -R a+rX' in memory
    for text in (ours, glim, fast):
        assert 'M6A3_SYNTHETIC_SMOKE' in text
        assert 'synthetic_smoke_contract.json' in text
        assert 'container_memory_evidence.sh' in text
        assert 'm6a5_write_container_memory_evidence' in text or \
            'm6a5_container_exit_trap' in text
        assert 'm6a7_start_process_rss_sampler' in text or \
            'm6a5_container_exit_trap' in text
        assert 'gt_mounted":"false' not in text
        assert 'gt_mounted":false' in text
        assert 'performance_run":false' in text
        assert 'm6a5_install_container_signal_traps' in text
    for text in (glim, fast):
        assert 'm6a7_start_process_rss_sampler' in text
        assert 'm6a5_write_container_memory_evidence' in text or \
            'm6a5_container_exit_trap' in text
    assert 'm6a7_start_process_rss_sampler' in ours
    assert 'RKO LIO Node is up!' in ours
    assert 'initialization end' in ours
    assert 'ros2 node list' in glim
    assert 'rosbag play --clock' in fast
    assert 'ROS_MASTER_URI=http://127.0.0.1:11311' in fast
    # The mapper's GNU-time report is complete only after children are reaped;
    # phase/memory evidence is finalized afterward in the shared EXIT path.
    assert fast.index('for child_pid in') < fast.index(
        'm6a5_write_container_memory_evidence')


def test_sampler_overhead_gate_is_preregistered_before_measurement():
    contract = json.loads(OVERHEAD_PREREGISTRATION.read_text())
    assert contract['status'] == 'preregistered_not_measured'
    assert contract['repeats'] == 20
    assert contract['order'] == 'AB_BA_alternating'
    assert contract['same_cpuset'] == '0-7'
    assert contract['same_image'] is True
    assert contract['measurement']['startup_excluded'] is True
    assert contract['measurement']['network'] == 'none'
    assert contract['measurement']['clock'] == 'monotonic'
    assert contract['measurement']['cpu_clock'] == 'resource.RUSAGE_SELF'
    assert contract['measurement']['sampler_interval_ms'] == 250
    assert contract['workload']['model'] == 'fixed_work_count_cpu'
    assert contract['workload']['worker_count'] == 8
    assert contract['bootstrap'] == {
        'confidence': 0.95, 'resamples': 100000, 'seed': 20260822,
        'statistic': 'mean_paired_wall_overhead_percent'}
    assert contract['gate'] == {
        'bootstrap_ci95_upper_percent_max': 5.0,
        'median_abs_overhead_percent_max': 2.0}


def test_sampler_overhead_v2_preregisters_low_priority_and_required_markers():
    contract = json.loads(OVERHEAD_PREREGISTRATION_V2.read_text())
    assert contract['schema_version'] == 2
    assert contract['status'] == 'preregistered_not_measured'
    assert contract['measurement']['sampler_interval_ms'] == 250
    assert contract['measurement']['sampler_scheduler_nice'] == 10
    assert contract['optimization'] == {
        'proc_enumeration': 'os.scandir',
        'process_rss_definition_unchanged': True,
        'self_exclusion_unchanged': True,
        'pid_reuse_checks_unchanged': True}
    assert contract['raw_run_markers'] == [
        'docker_exit_status.txt', 'mode.txt', 'workload_time.json']
    assert contract['aggregation']['missing_marker_policy'] == \
        'fail_closed_before_numeric_aggregation'
    assert contract['repeats'] == 20
    assert contract['order'] == 'AB_BA_alternating'
    assert contract['same_cpuset'] == '0-7'
    assert contract['same_image'] is True


def test_sampler_overhead_v3_preregisters_cross_sample_pid_guard():
    contract = json.loads(OVERHEAD_PREREGISTRATION_V3.read_text())
    assert contract['schema_version'] == 3
    assert contract['status'] == 'preregistered_not_measured'
    assert contract['measurement']['sampler_interval_ms'] == 250
    assert contract['measurement']['sampler_scheduler_nice'] == 10
    assert contract['optimization']['proc_enumeration'] == 'os.scandir'
    assert contract['optimization']['pid_start_time_guard'] == \
        'cross_sample_single_stat_read'
    assert contract['optimization']['process_rss_definition_unchanged'] is True
    assert contract['optimization']['self_exclusion_unchanged'] is True
    assert contract['optimization']['pid_reuse_checks_unchanged'] is True
    assert contract['raw_run_markers'] == [
        'docker_exit_status.txt', 'mode.txt', 'workload_time.json']


def test_ours_image_archive_label_mismatch_is_fail_closed(monkeypatch):
    receipt = _receipt()
    digest = receipt['systems']['ours']['container']['image_digest']
    labels = {
        'benchmark.ours.revision': receipt['systems']['ours']['repository']['revision'],
        'benchmark.rko_lio.initialized': 'true',
        'benchmark.rko_lio.gitlink_revision': '622b74778a41f753d47aa5918043755ebcbd4c75',
        'benchmark.ndt_omp.submodule_revision': '497411279593eb261a3e3d04cdcbb4717af33ca3',
        'benchmark.rko_lio.archive_sha256': '0' * 64,
    }

    class Completed:
        stdout = json.dumps([{'Id': digest, 'Config': {'Labels': labels}}])

    monkeypatch.setattr(RUNNER.subprocess, 'run', lambda *_a, **_k: Completed())
    with pytest.raises(RUNNER.ContractError, match='archive_sha256'):
        RUNNER.image_ref_and_labels('ours', receipt, inspect=True)


def test_fast_runner_and_entrypoint_use_owned_workspace():
    runner = (ROOT / 'scripts/run_fast_livo2_benchmark.py').read_text()
    entrypoint = (ROOT / 'scripts/fast_livo2_container_run.sh').read_text()
    assert 'fast-livo2-benchmark:ros1-pinned' in runner
    assert '/opt/fast_livo_ws/devel/setup.bash' in entrypoint
    assert 'hdl_localization_noetic:local' not in entrypoint
