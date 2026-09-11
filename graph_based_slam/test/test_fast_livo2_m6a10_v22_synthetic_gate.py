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


import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'm6a10_v22_synthetic_gate', ROOT / 'scripts/run_fast_livo2_m6a10_v22_synthetic_gate.py'
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _root(tmp_path: Path) -> Path:
    root = tmp_path / 'fresh'
    root.mkdir()
    return root


def test_build_argv_is_digest_bound_single_rw_out_and_input_free(tmp_path):
    root = _root(tmp_path)
    sources = MODULE._fake_sources(root, 'success')
    argv = MODULE.build_docker_argv(
        root, 'success', 'm6a10-v22-synthetic-success', sources['feeder']['sha256']
    )
    MODULE.validate_docker_argv(argv, 'm6a10-v22-synthetic-success')
    mounts = [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == '--mount']
    assert sum(item.endswith('dst=/out,readonly=false') for item in mounts) == 1
    assert all('dst=/input' not in item for item in mounts)
    assert '--read-only' in argv
    assert argv[argv.index('--network') + 1] == 'none'
    assert '--rm' not in argv
    assert argv.count('--tmpfs') == 2
    assert argv[-2:] == [MODULE.IMAGE_ID, '/runner/v22_runtime.sh']


def test_argv_rejects_rw_output_tmpfs_and_input_mount(tmp_path):
    root = _root(tmp_path)
    sources = MODULE._fake_sources(root, 'success')
    argv = MODULE.build_docker_argv(
        root, 'success', 'm6a10-v22-synthetic-success', sources['feeder']['sha256']
    )
    bad_tmpfs = list(argv)
    bad_tmpfs[bad_tmpfs.index('/tmp:rw,nosuid,nodev')] = '/out:rw,nosuid,nodev'
    with pytest.raises(MODULE.GateError, match='tmpfs'):
        MODULE.validate_docker_argv(bad_tmpfs, 'm6a10-v22-synthetic-success')
    bad_mount = list(argv)
    mount_index = next(index for index, item in enumerate(bad_mount) if item == '--mount')
    bad_mount[mount_index + 1] = 'type=bind,src=/tmp/input,dst=/input/ntu_viral.bag,readonly'
    with pytest.raises(MODULE.GateError, match='input'):
        MODULE.validate_docker_argv(bad_mount, 'm6a10-v22-synthetic-success')


def test_inspect_requires_stopped_oom_false_and_one_rw_out():
    record = {
        'Name': '/m6a10-v22-synthetic-success',
        'Image': MODULE.IMAGE_ID,
        'State': {'Status': 'exited', 'OOMKilled': False, 'ExitCode': 0},
        'HostConfig': {'NetworkMode': 'none', 'ReadonlyRootfs': True},
        'Mounts': [
            {'Destination': '/runner/v22_runtime.sh', 'RW': False},
            {'Destination': '/runner/fakebin', 'RW': False},
            {'Destination': '/runner/synthetic/feeder.py', 'RW': False},
            {'Destination': '/out', 'RW': True},
        ],
    }
    assert MODULE._inspect_shape([record], 'm6a10-v22-synthetic-success', 0)['Name'].startswith(
        '/'
    )
    record['State']['OOMKilled'] = True
    with pytest.raises(MODULE.GateError, match='OOM'):
        MODULE._inspect_shape([record], 'm6a10-v22-synthetic-success', 0)


def test_fake_sources_have_success_and_failure_contracts(tmp_path):
    # Use independent fresh roots because source sealing is intentionally
    # no-overwrite.
    success_root = tmp_path / 'success'
    success_root.mkdir()
    success = MODULE._fake_sources(success_root, 'success')
    failure_root = tmp_path / 'failure'
    failure_root.mkdir()
    failure = MODULE._fake_sources(failure_root, 'failure')
    assert success['feeder']['sha256'] != failure['feeder']['sha256']
    assert 'status' in (success_root / 'synthetic/feeder.py').read_text()
    assert 'raise SystemExit(17)' in (failure_root / 'synthetic/feeder.py').read_text()
    assert 'callback_consumer_evidence.json' in (success_root / 'fakebin/rosservice').read_text()


def test_receipt_paths_and_safety_contract_are_source_pinned():
    text = (ROOT / 'scripts/run_fast_livo2_m6a10_v22_synthetic_gate.py').read_text()
    assert 'formal_replay_started' in text
    assert 'input_mount_count' in text
    assert 'readonly=false' in text
    assert '"docker", "rm"' in text
    assert 'docker rm --force' not in text
    assert 'ground_truth_content_opened' in text
    assert 'scorer_invoked' in text
    assert 'map_saved' in text
