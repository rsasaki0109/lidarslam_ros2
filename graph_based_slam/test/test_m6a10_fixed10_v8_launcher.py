#!/usr/bin/env python3
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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_m6a10_fixed10_v8.py'
SPEC = importlib.util.spec_from_file_location('m6a10_fixed10_v8', SCRIPT)
V8 = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = V8
SPEC.loader.exec_module(V8)


def test_materializer_tree_hash_known_path_size_content_digest(tmp_path):
    tree = tmp_path / 'tree'
    (tree / 'sub').mkdir(parents=True)
    (tree / 'a.txt').write_bytes(b'abc')
    (tree / 'sub' / 'b.txt').write_bytes(b'z')
    assert V8.TREE_HASH_KIND == 'relative_path_size_content_sha256_v1'
    assert V8.tree_sha256(tree) == (
        '62a6652bfe7117f13f6738aeecf1464d4b570023638f7e8368be926a6b533b16')
    config = V8.LaunchConfig(root=tmp_path / 'attempt', repo_root=tmp_path)
    assert config.expected_input_tree_sha256 == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
    assert V8._file_sha256(V8.MATERIALIZER_HELPER_PATH) == (
        'caddcf0ae85d74444ae65ea85ed33d5e561a2569dc8ef180b2496a9d87c132c9')


def test_v8_identity_failure_uses_v8_receipt_without_starting_runner(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('fixture\n', encoding='utf-8')
    config = V8.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root,
        expected_input_tree_sha256=V8.tree_sha256(input_root))
    events = []

    def identity(_config):
        events.append('identity')
        return {'fixture': True}

    def failed_quiescence(_config, path):
        events.append('quiescence')
        path.write_text(json.dumps({'status': 'FAIL_CLOSED'}), encoding='utf-8')
        return 1

    def should_not_start(*_args, **_kwargs):
        pytest.fail('v8 runner must not start after failed preflight')

    assert V8.run_v8(
        config, identity_validator=identity,
        quiescence_runner=failed_quiescence,
        process_factory=should_not_start) == 10
    assert events == ['identity', 'quiescence']
    marker = json.loads((config.root / 'attempt_root_marker.json').read_text())
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert marker['contract_id'] == V8.CONTRACT_ID
    assert marker['launcher_path'].endswith('run_m6a10_fixed10_v8.py')
    assert closure['contract_id'] == V8.CONTRACT_ID
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert not (config.root / 'launch_attempt.json').exists()
