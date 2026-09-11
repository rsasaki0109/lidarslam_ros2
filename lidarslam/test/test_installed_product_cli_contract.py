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

"""Static contract tests for the curated installed product CLI."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / 'lidarslam'
RUNTIME_MANIFEST = PACKAGE_ROOT / 'product-runtime-files.txt'
INSTALL_CHECK = REPO_ROOT / 'scripts' / 'check_installed_product_cli.py'
LAUNCHER = REPO_ROOT / 'scripts' / 'lidarslam'


def _runtime_names() -> list[str]:
    return [
        line.strip()
        for line in RUNTIME_MANIFEST.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def _load_install_check():
    spec = importlib.util.spec_from_file_location(
        'check_installed_product_cli',
        INSTALL_CHECK,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_probe_cli(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.with_name('launcher_probe_module.py').write_text(
        'VALUE = 1\n',
        encoding='utf-8',
    )
    path.write_text(
        'import launcher_probe_module, os, sys\n'
        'assert launcher_probe_module.VALUE == 1\n'
        'print(os.environ.get("LIDARSLAM_LAUNCHER_SETUP_TEST", "missing"))\n'
        'print(os.environ.get("LIDARSLAM_CLI_NAME", "missing"))\n'
        'print(os.environ.get("PYTHONDONTWRITEBYTECODE", "missing"))\n'
        'print(" ".join(sys.argv[1:]))\n',
        encoding='utf-8',
    )


def _run_launcher(path: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop('LIDARSLAM_LAUNCHER_SETUP_TEST', None)
    env.pop('PYTHONDONTWRITEBYTECODE', None)
    return subprocess.run(
        [str(path), 'doctor', '/tmp/example bag', '--json'],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )


def test_installed_help_validator_enforces_source_contract(tmp_path: Path):
    module = _load_install_check()

    module._validate_installed_help(
        REPO_ROOT / 'scripts' / 'lidarslam',
        tmp_path,
    )


def test_direct_installed_launcher_activates_merged_prefix(tmp_path: Path):
    prefix = tmp_path / 'merged install'
    launcher = prefix / 'bin' / 'lidarslam-map'
    launcher.parent.mkdir(parents=True)
    shutil.copy2(LAUNCHER, launcher)
    _write_probe_cli(
        prefix / 'share/lidarslam/product/scripts/lidarslam_cli.py'
    )
    (prefix / 'setup.bash').write_text(
        'export LIDARSLAM_LAUNCHER_SETUP_TEST=merged\n',
        encoding='utf-8',
    )

    result = _run_launcher(launcher)

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'merged',
        'lidarslam-map',
        '1',
        'doctor /tmp/example bag --json',
    ]

    shortcut = tmp_path / 'shortcut' / 'lidarslam-map'
    shortcut.parent.mkdir()
    shortcut.symlink_to(launcher)
    shortcut_result = _run_launcher(shortcut)
    assert shortcut_result.returncode == 0, shortcut_result.stderr
    assert shortcut_result.stdout.splitlines() == result.stdout.splitlines()
    assert not list(prefix.rglob('__pycache__'))


def test_direct_installed_launcher_activates_isolated_workspace(tmp_path: Path):
    install_root = tmp_path / 'isolated install'
    prefix = install_root / 'lidarslam'
    launcher = prefix / 'bin' / 'lidarslam-map'
    launcher.parent.mkdir(parents=True)
    shutil.copy2(LAUNCHER, launcher)
    _write_probe_cli(
        prefix / 'share/lidarslam/product/scripts/lidarslam_cli.py'
    )
    (install_root / 'setup.bash').write_text(
        'export LIDARSLAM_LAUNCHER_SETUP_TEST=isolated\n',
        encoding='utf-8',
    )

    result = _run_launcher(launcher)

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'isolated',
        'lidarslam-map',
        '1',
        'doctor /tmp/example bag --json',
    ]


def test_source_launcher_activates_only_a_matching_built_workspace(
    tmp_path: Path,
):
    workspace = tmp_path / 'source workspace'
    repo = workspace / 'src' / 'lidar_slam_ros2'
    scripts = repo / 'scripts'
    scripts.mkdir(parents=True)
    launcher = scripts / 'lidarslam'
    shutil.copy2(LAUNCHER, launcher)
    _write_probe_cli(scripts / 'lidarslam_cli.py')

    install_root = workspace / 'install'
    installed_scripts = (
        install_root / 'lidarslam/share/lidarslam/product/scripts'
    )
    _write_probe_cli(installed_scripts / 'lidarslam_cli.py')
    (install_root / 'setup.bash').write_text(
        'export LIDARSLAM_LAUNCHER_SETUP_TEST=source\n',
        encoding='utf-8',
    )

    result = _run_launcher(launcher)

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'source',
        'lidarslam',
        '1',
        'doctor /tmp/example bag --json',
    ]

    (installed_scripts / 'lidarslam_cli.py').unlink()
    (installed_scripts / 'launcher_probe_module.py').unlink()
    installed_scripts.rmdir()
    result_without_product = _run_launcher(launcher)
    assert result_without_product.returncode == 0, result_without_product.stderr
    assert result_without_product.stdout.splitlines()[0] == 'missing'


def test_direct_launcher_reports_setup_activation_failure(tmp_path: Path):
    prefix = tmp_path / 'broken install'
    launcher = prefix / 'bin' / 'lidarslam-map'
    launcher.parent.mkdir(parents=True)
    shutil.copy2(LAUNCHER, launcher)
    _write_probe_cli(
        prefix / 'share/lidarslam/product/scripts/lidarslam_cli.py'
    )
    (prefix / 'setup.bash').write_text('return 23\n', encoding='utf-8')

    result = _run_launcher(launcher)

    assert result.returncode == 70
    assert 'failed to activate lidarslam setup' in result.stderr
    assert result.stdout == ''


def test_product_runtime_manifest_is_curated_and_complete():
    names = _runtime_names()

    assert names
    assert len(names) == len(set(names))
    assert 'lidarslam_guided.py' in names
    assert 'lidarslam_cli.py' in names
    assert 'first_map_demo.py' in names
    assert 'product_schema.py' in names
    assert 'product_profiles.py' in names
    assert 'session_history.py' in names
    assert 'session_compare.py' in names
    assert 'support_bundle.py' in names
    assert 'migrate_run_manifest.py' in names
    assert 'plan_image_rollback.py' in names
    assert 'run_autoware_map_from_bag.py' in names
    assert 'view_autoware_map.py' in names
    assert 'apply_map_edit.py' in names
    assert 'map_edit.py' in names
    assert 'merge_map_sessions.py' in names
    assert 'map_merge.py' in names
    assert 'record_backend_input.sh' in names
    assert 'run_offline_determinism_check.sh' in names
    assert 'run_map_soak.py' in names
    assert 'verify_autoware_map.py' in names
    assert 'verify_map_bundle.py' in names
    assert 'first_map_validation_receipt.py' in names
    assert 'create_first_map_validation_receipt.py' in names
    assert 'run_first_map_demo.sh' in names
    assert 'run_docker_demo.sh' in names
    assert 'download_mid360_robot_public_dataset.py' in names
    assert 'mid360_robot_public_datasets.py' in names
    assert 'gaussian_splatting_train.py' not in names
    for name in names:
        assert Path(name).name == name
        assert (REPO_ROOT / 'scripts' / name).is_file(), name


def test_cmake_preserves_historical_node_and_installs_distinct_cli_names():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'product-runtime-files.txt' in cmake
    assert 'DESTINATION bin' in cmake
    assert 'RENAME lidarslam-map' in cmake
    assert 'DESTINATION lib/${PROJECT_NAME}' in cmake
    assert 'RENAME lidarslam-cli' in cmake
    assert 'scripts/completions/lidarslam-map.bash' in cmake
    assert 'DESTINATION share/${PROJECT_NAME}/product/completions' in cmake
    assert 'release-image-v1.schema.json' in cmake
    assert 'rollback-plan-v1.schema.json' in cmake
    assert 'sensor-setup-v1.schema.json' in cmake
    assert 'sensor-setup-rejection-v1.schema.json' in cmake
    assert 'map-session-recovery-v1.schema.json' in cmake
    assert 'map-session-index-v1.schema.json' in cmake
    assert 'map-session-catalog-v1.schema.json' in cmake
    assert 'map-session-comparison-v1.schema.json' in cmake
    assert 'support-bundle-v1.schema.json' in cmake
    assert 'map-edit-plan-v1.schema.json' in cmake
    assert 'map-edit-receipt-v1.schema.json' in cmake
    assert 'first-map-validation-receipt-v1.schema.json' in cmake
    assert 'first-map-handoff-v1.schema.json' in cmake
    assert 'first-map-demo-plan-v1.schema.json' in cmake
    assert 'DESTINATION share/${PROJECT_NAME}/product/schemas' in cmake
    assert 'generate_product_build_info.py' in cmake
    assert 'product-build-info.json' in cmake
    assert 'LIDARSLAM_SOURCE_REVISION' in cmake
    assert 'LIDARSLAM_SOURCE_DIRTY' in cmake
    assert 'install(TARGETS\n  lidarslam' in cmake
    assert 'PATTERN "__pycache__" EXCLUDE' in cmake
    assert 'PATTERN "*.pyc" EXCLUDE' in cmake
    assert 'PATTERN "*.pyo" EXCLUDE' in cmake


def test_primary_workflows_resolve_source_and_installed_runtime_layouts():
    workflow_names = (
        'run_rko_lio_graph_autoware_dogfood.sh',
        'run_open_data_gnss_smoke.sh',
        'run_open_data_applanix_velodyne_gnss_smoke.sh',
    )

    for name in workflow_names:
        script = (REPO_ROOT / 'scripts' / name).read_text(encoding='utf-8')

        assert 'PACKAGE_SHARE=' in script, name
        assert 'WORK_ROOT=' in script, name
        assert 'WORKSPACE_SETUP=' in script, name
        assert '${PACKAGE_SHARE}/param/' in script, name
        assert '${WORK_ROOT}/output/' in script, name
        assert '${REPO_ROOT}/scripts/' not in script, name

    dogfood = (REPO_ROOT / 'scripts' / workflow_names[0]).read_text(
        encoding='utf-8'
    )
    assert 'PATH_TO_TUM_SCRIPT="${SCRIPT_DIR}/path_to_tum.py"' in dogfood
    assert 'ODOM_TO_TUM_SCRIPT="${SCRIPT_DIR}/odom_to_tum.py"' in dogfood
    assert 'APE_FROM_TUM_SCRIPT="${SCRIPT_DIR}/ape_from_tum.py"' in dogfood
    assert '"$SCRIPT_DIR/simple_lanelet2_generator.py"' in dogfood

    for name in workflow_names[1:]:
        script = (REPO_ROOT / 'scripts' / name).read_text(encoding='utf-8')
        assert '"${SCRIPT_DIR}/verify_autoware_map.py"' in script, name
