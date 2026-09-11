# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Static and isolated-runtime contracts for the slim Docker product."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = REPO_ROOT / 'Dockerfile'
DOCKERIGNORE = REPO_ROOT / '.dockerignore'
ENTRYPOINT = REPO_ROOT / 'docker' / 'entrypoint.sh'
RUNTIME_MANIFEST = REPO_ROOT / 'lidarslam' / 'product-runtime-files.txt'
SCRIPT_DIR = REPO_ROOT / 'scripts'
RUNTIME_STAGE = 'FROM ros:${ROS_DISTRO}-ros-core AS runtime'
RUNTIME_DEMO_FILES = (
    'run_first_map_demo.sh',
    'run_docker_demo.sh',
    'download_mid360_robot_public_dataset.py',
    'mid360_robot_public_datasets.py',
)
DEPENDENCY_MANIFESTS = (
    'lidarslam/package.xml',
    'lidarslam_msgs/package.xml',
    'scanmatcher/package.xml',
    'graph_based_slam/package.xml',
    'Thirdparty/ndt_omp_ros2/package.xml',
    'Thirdparty/rko_lio/package.xml',
)


def _runtime_names() -> set[str]:
    return {
        line.strip()
        for line in RUNTIME_MANIFEST.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    }


def test_runtime_docker_stage_is_source_free_and_fail_closed():
    """The final image receives installed product bytes, not build inputs."""
    dockerfile = DOCKERFILE.read_text(encoding='utf-8')

    assert 'FROM ros:${ROS_DISTRO}-ros-core AS builder' in dockerfile
    assert dockerfile.count(RUNTIME_STAGE) == 1
    builder, runtime = dockerfile.split(RUNTIME_STAGE, 1)

    assert builder.count('COPY . .') == 1
    assert 'collect_runtime_apt_packages.py' in builder
    assert '--output /tmp/lidarslam-runtime-packages.txt' in builder
    assert 'COPY . .' not in runtime
    assert 'colcon build' not in runtime
    assert 'python3-colcon-common-extensions' not in runtime
    assert 'python3-rosdep' not in runtime
    assert '\n    git \\' not in runtime
    assert 'COPY --from=builder /lidarslam_ws/install/' in runtime
    assert (
        'COPY docker/entrypoint.sh /lidarslam_ws/docker/entrypoint.sh'
        in runtime
    )
    assert 'test -s /tmp/lidarslam-runtime-packages.txt' in runtime
    assert 'xargs -r apt-get install -y --no-install-recommends' in runtime
    assert 'DEMO_DATA_DIR=/lidarslam_ws/datasets/mid360_public' in runtime
    assert 'DEMO_OUTPUT_DIR=/lidarslam_ws/output/mid360_demo' in runtime
    assert builder.count('lidarslam-map start --help') == 1
    assert runtime.count('lidarslam-map start --help') == 1
    assert dockerfile.count("grep -Fq 'report <output>'") == 2
    assert dockerfile.count("grep -Fq 'Detect and configure the sensors'") == 2
    assert dockerfile.count("grep -Fq -- '--map-output-dir'") == 2
    assert (
        'CMD ["bash", "/lidarslam_ws/install/lidarslam/share/lidarslam/'
        'product/scripts/run_docker_demo.sh"]'
    ) in runtime


def test_builder_dependency_layer_is_keyed_by_package_manifests():
    """Source-only edits must not invalidate the expensive rosdep layer."""
    dockerfile = DOCKERFILE.read_text(encoding='utf-8')
    builder, _ = dockerfile.split(RUNTIME_STAGE, 1)
    rosdep = builder.index('rosdep install -r -y')
    source_copy = builder.index('COPY . .')
    colcon = builder.index('colcon build')

    for manifest in DEPENDENCY_MANIFESTS:
        copy = f'COPY {manifest} {manifest}'
        assert builder.count(copy) == 1
        assert builder.index(copy) < rosdep
    assert rosdep < source_copy < colcon


def test_docker_context_excludes_nested_submodule_git_metadata():
    """Linked-worktree gitdir pointers must not poison the source cache key."""
    patterns = {
        line.strip()
        for line in DOCKERIGNORE.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    }

    assert '.git' in patterns
    assert '**/.git' in patterns


def test_entrypoint_enables_nounset_after_ros_setup_files():
    """ROS setup files must load before the wrapper enables nounset."""
    entrypoint = ENTRYPOINT.read_text(encoding='utf-8')

    assert 'set -eo pipefail' in entrypoint
    assert 'set -euo pipefail' not in entrypoint
    ros_setup = entrypoint.index(
        'source "/opt/ros/${ROS_DISTRO}/setup.bash"'
    )
    product_setup = entrypoint.index(
        'source /lidarslam_ws/install/setup.bash'
    )
    nounset = entrypoint.index('set -u')
    execute = entrypoint.index('exec "$@"')
    assert ros_setup < product_setup < nounset < execute


def test_curated_runtime_contains_source_independent_demo_files():
    """Every default-demo file is installed without source-only imports."""
    names = _runtime_names()
    assert set(RUNTIME_DEMO_FILES).issubset(names)

    downloader = (
        SCRIPT_DIR / 'download_mid360_robot_public_dataset.py'
    ).read_text(encoding='utf-8')
    datasets = (SCRIPT_DIR / 'mid360_robot_public_datasets.py').read_text(
        encoding='utf-8'
    )
    first_map = (SCRIPT_DIR / 'run_first_map_demo.sh').read_text(
        encoding='utf-8'
    )
    assert 'mid360_robot_tools' not in downloader
    assert 'mid360_robot_tools' not in datasets
    assert 'def payload_to_json(' in datasets
    assert '${SCRIPT_DIR}/download_mid360_robot_public_dataset.py' in first_map
    assert '${REPO_ROOT}/scripts/' not in first_map


def test_downloader_runs_from_an_isolated_installed_layout(tmp_path: Path):
    """The downloader imports and lists datasets without a source checkout."""
    installed_scripts = tmp_path / 'prefix' / 'product' / 'scripts'
    installed_scripts.mkdir(parents=True)
    for name in RUNTIME_DEMO_FILES[2:]:
        shutil.copy2(SCRIPT_DIR / name, installed_scripts / name)

    work_dir = tmp_path / 'work'
    work_dir.mkdir()
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    result = subprocess.run(
        [
            sys.executable,
            str(installed_scripts / 'download_mid360_robot_public_dataset.py'),
            '--list',
            '--json',
        ],
        check=False,
        capture_output=True,
        cwd=work_dir,
        env=env,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    ids = {dataset['id'] for dataset in payload['datasets']}
    assert 'driving_slam_mid360' in ids
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in ids
