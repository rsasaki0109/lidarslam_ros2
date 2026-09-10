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


"""
Static gates for the v12 Docker/build candidate.

No Docker daemon, image build, ROS runtime, input, ground truth, or scorer is
used here.  The recipe is checked as an exact v11-lineage source candidate.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v12.Dockerfile'
BUILD = ROOT / 'scripts/build_fast_livo2_m6a10_v12_image.sh'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml'
BASE_TAG = 'm6a10-v2c-v11-bounded-end-gap-20260823t121749z-fast-livo2-benchmark:ros1-pinned'
BASE_ID = 'sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a'
CONTRACT = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
TRANSPORT = 'm6a10-v12-callback-ack-transport-outstanding-v1'
PATCH_SHA = '39c77535a7557365dac6b0f2c99849038b29670a57c3101be3a39138317c6333'
TERMINAL_SHA = '1b42ed713dbb2a8614902ae6214d5790ab9c73ac59792f4d68753320cef47a91'
CONSUMER_SHA = '02078640496daa28f0be471355c6e2ad2822a76199cda1b54e738eeff5fe48de'
WRAPPER_SHA = '32fce2c054b92f0d695fd4d32c7737ab75dd3a467fdee1c579c82c63398894e6'
STUB_ROS_SHA = '40c3811b6225da6ffdb2c98ed86b9d218f168ef7c1c0294825f7ad216387bb0a'
STUB_TRIGGER_SHA = '34704c7662d96d395972ac2cc3a1a874331055bb48c5b16fdfb2494f5145dc24'


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_v12_sources_are_exact_and_v11_base_is_pinned():
    assert (
        _sha(ROOT / 'docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch')
        == PATCH_SHA
    )
    assert _sha(ROOT / 'tools/m6a10_terminal_support_context_v12_selftest.cpp') == TERMINAL_SHA
    assert _sha(ROOT / 'tools/m6a10_consumer_evidence_v12_selftest.cpp') == CONSUMER_SHA
    assert _sha(ROOT / 'tools/m6a10_v12_test_stubs/ros/ros.h') == STUB_ROS_SHA
    assert _sha(ROOT / 'tools/m6a10_v12_test_stubs/std_srvs/Trigger.h') == STUB_TRIGGER_SHA
    assert _sha(ROOT / 'scripts/fast_livo2_m6a10_v12_formal_container_run.sh') == WRAPPER_SHA
    assert DOCKERFILE.is_file() and BUILD.is_file()


def test_v12_docker_recipe_applies_delta_compiles_runs_gates_then_catkin():
    text = DOCKERFILE.read_text(encoding='utf-8')
    assert f'ARG FAST_LIVO2_V11_BASE_IMAGE={BASE_TAG}' in text
    assert f'ARG FAST_LIVO2_V11_EXPECTED_BASE_ID={BASE_ID}' in text
    assert 'FROM ${FAST_LIVO2_V11_BASE_IMAGE}' in text
    for source in (
        'docker/patches/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch',
        'tools/m6a10_terminal_support_context_v12_selftest.cpp',
        'tools/m6a10_consumer_evidence_v12_selftest.cpp',
        'tools/m6a10_v12_test_stubs',
        'scripts/fast_livo2_m6a10_v12_formal_container_run.sh',
    ):
        assert f'COPY {source}' in text
    for expected in (
        PATCH_SHA,
        TERMINAL_SHA,
        CONSUMER_SHA,
        STUB_ROS_SHA,
        STUB_TRIGGER_SHA,
        WRAPPER_SHA,
    ):
        assert expected not in text  # values arrive as pinned build args
    assert 'sha256sum /tmp/fast_livo2.m6a10-v2c-v12-nonlidar-boundary-transport.patch' in text
    assert 'sha256sum /tmp/m6a10_terminal_support_context_v12_selftest.cpp' in text
    assert 'sha256sum /tmp/m6a10_consumer_evidence_v12_selftest.cpp' in text
    assert 'sha256sum /tmp/m6a10_v12_test_stubs/ros/ros.h' in text
    assert 'sha256sum /tmp/m6a10_v12_test_stubs/std_srvs/Trigger.h' in text
    assert 'sha256sum /tmp/fast_livo2_m6a10_v12_formal_container_run.sh' in text
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --check --recount' in text
    assert 'git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount' in text
    assert CONTRACT in text and TRANSPORT in text
    assert 'transport_contract_version' in text
    assert "'transport_contract_version'" in text
    assert 'g++ -std=c++17 -Wall -Wextra -Werror' in text
    assert '$(pkg-config --libs roscpp std_srvs)' in text
    assert '/tmp/m6a10_terminal_support_context_v12_selftest' in text
    assert '/tmp/m6a10_consumer_evidence_v12_selftest' in text
    assert 'catkin_make -DCMAKE_BUILD_TYPE=Release -j2' in text
    assert (
        text.index('git -C /opt/fast_livo_ws/src/FAST-LIVO2 apply --recount')
        < text.index('g++ -std=c++17')
        < text.index('catkin_make')
    )
    assert 'benchmark.fast_livo2.m6a10_ground_truth_present="false"' in text
    assert 'benchmark.fast_livo2.m6a10_scorer_present="false"' in text
    assert 'benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"' in text
    assert 'benchmark.fast_livo2.m6a10_network_expectation="none"' in text
    assert 'docker run' not in text.lower()
    assert 'rosbag' not in text.lower()


def test_v12_build_script_is_network_isolated_unique_and_receipt_immutable():
    text = BUILD.read_text(encoding='utf-8')
    assert BUILD.stat().st_mode & 0o111
    assert BASE_TAG in text and BASE_ID in text
    for expected in (PATCH_SHA, TERMINAL_SHA, CONSUMER_SHA, STUB_ROS_SHA, STUB_TRIGGER_SHA):
        assert expected in text
    assert 'EXPECTED_WRAPPER_SHA256="$(sha256sum "${WRAPPER_PATH}"' in text
    assert CONTRACT in text and TRANSPORT in text
    assert 'IMAGE_TAG="${FAST_LIVO2_V12_IMAGE_TAG:-m6a10-v2c-v12-' in text
    assert text.count('docker build --network none --pull=false') == 1
    assert '--build-arg "FAST_LIVO2_V11_BASE_ID=${BASE_ID}"' in text
    assert '--build-arg "FAST_LIVO2_M6A10_V12_PATCH_SHA256=${EXPECTED_PATCH_SHA256}"' in text
    assert '--build-arg "FAST_LIVO2_M6A10_V12_WRAPPER_SHA256=${EXPECTED_WRAPPER_SHA256}"' in text
    assert 'test ! -e "${RECEIPT_PATH}"' in text
    assert 'test ! -e "${SIDECAR_PATH}"' in text
    assert 'chmod 0444 "${RECEIPT_PATH}" "${SIDECAR_PATH}"' in text
    assert 'formal_replay_forbidden' in text
    assert 'ground_truth_content_opened' in text and 'scorer_invoked' in text
    assert 'docker run' not in text.lower()
    assert 'rosbag' not in text.lower()
    result = subprocess.run(
        ['bash', '-n', str(BUILD)], cwd=ROOT, capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_v12_profile_records_build_not_run_recipe_and_source_hashes():
    document = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    candidate = document['competitive_slam_profile']['m6a10_fast_livo2_v2c_v12_formal_ready']
    source = candidate['source']
    assert source['v12_dockerfile_path'] == 'docker/fast_livo2_m6a10_v12.Dockerfile'
    assert source['v12_build_script_path'] == 'scripts/build_fast_livo2_m6a10_v12_image.sh'
    assert source['v12_delta_patch_sha256'] == PATCH_SHA
    assert source['v12_selftest_sha256'] == TERMINAL_SHA
    assert source['v12_consumer_selftest_sha256'] == CONSUMER_SHA
    assert source['v12_wrapper_path'] == 'scripts/fast_livo2_m6a10_v12_formal_container_run.sh'
    assert source['v12_build_status'] == 'RECIPE_STATIC_VALIDATED_NOT_BUILT'
    assert candidate['execution']['status'] == 'build_not_run'
    assert candidate['execution']['image_tag'] is None
    assert candidate['execution']['image_id'] is None
    assert candidate['execution']['ground_truth_content_opened'] is False
    assert candidate['execution']['scorer_invoked'] is False
    assert candidate['execution']['formal_replay_started'] is False
