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


"""Host-only tests for the v13 output-persistence gate."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v13_persistence_gate.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v13_persistence', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _mount_specs(argv):
    return [argv[index + 1]
            for index, item in enumerate(argv[:-1]) if item == '--mount']


def test_lineage_and_profile_are_pinned_without_feeder_root_cause_claim():
    profile = MODULE.PROFILE_PATH.read_text(encoding='utf-8')
    assert MODULE.sha256_file(MODULE.PROFILE_PATH) == MODULE.PROFILE_SHA256
    assert 'feeder_underlying_nonzero_cause: unknown' in profile
    assert 'output_loss_mechanism: duplicate_out_tmpfs_before_bind' in profile
    assert 'formal_replay_authorized: false' in profile
    assert 'input_mounts: 0' in profile


def test_argv_has_single_rw_output_bind_and_no_input_or_output_tmpfs(tmp_path):
    root = tmp_path / 'fresh'
    root.mkdir()
    (root / 'out').mkdir()
    argv = MODULE.build_persistence_argv(root)
    mounts = _mount_specs(argv)
    destinations = [MODULE._mount_destination(spec) for spec in mounts]
    assert destinations == ['/out', MODULE.WRAPPER_DESTINATION]
    assert mounts[0] == 'type=bind,src=%s,dst=/out,readonly=false' % (
        root / 'out').resolve()
    assert mounts[1].endswith(',dst=%s,readonly' % MODULE.WRAPPER_DESTINATION)
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--pull=never' in argv and '--init' in argv
    assert '--rm' not in argv
    assert not any(item == '--tmpfs' and argv[index + 1].split(':', 1)[
                   0] == '/out' for index, item in enumerate(argv[:-1]))
    assert not any('/input' in item or 'bag' in item.lower()
                   for item in mounts)
    assert argv[-5] == '--entrypoint'
    assert argv[-4] == '/bin/sh'
    assert argv[-3] == MODULE.IMAGE_ID
    assert argv[-2] == '-ceu'
    assert argv[-1] == MODULE.PAYLOAD


def test_payload_writes_exact_marker_stderr_and_receipt(tmp_path):
    output = tmp_path / 'out'
    output.mkdir()
    for name, payload in (
        ('persistence.marker', MODULE.MARKER_BYTES),
        ('feeder.stderr', MODULE.STDERR_BYTES),
        ('feeder_receipt.json', MODULE.RECEIPT_BYTES),
    ):
        (output / name).write_bytes(payload)
    observed = MODULE.validate_persisted_output(output)
    assert set(observed) == {
        'persistence.marker',
        'feeder.stderr',
        'feeder_receipt.json'}
    assert json.loads(MODULE.RECEIPT_BYTES.decode()) == {
        'contract': MODULE.CONTRACT,
        'formal_replay_started': False,
        'status': 'PASS',
    }
    (output / 'feeder.stderr').write_bytes(b'tampered\n')
    with pytest.raises(MODULE.GateError, match='exact bytes'):
        MODULE.validate_persisted_output(output)


def test_sealing_refuses_overwrite_and_symlink(tmp_path):
    target = tmp_path / 'receipt.json'
    assert MODULE._atomic_create(target, b'first\n')
    with pytest.raises(MODULE.GateError, match='overwrite'):
        MODULE._atomic_create(target, b'second\n')
    link = tmp_path / 'link'
    link.symlink_to(target)
    with pytest.raises(MODULE.GateError, match='overwrite'):
        MODULE._atomic_create(link, b'unsafe\n')


def test_source_does_not_describe_feeder_root_cause_as_fixed():
    text = SCRIPT.read_text(encoding='utf-8')
    assert 'underlying nonzero cause remains unknown' in text
    assert 'output-loss mechanism' in text
    assert '"docker", "run"' in text
    assert '--tmpfs", "/out' not in text
