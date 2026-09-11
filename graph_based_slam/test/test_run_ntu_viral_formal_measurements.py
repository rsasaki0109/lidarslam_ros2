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

import argparse
import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'formal_runner', ROOT / 'scripts/run_ntu_viral_formal_measurements.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_formal_commands_preserve_track_modalities_and_repetitions(tmp_path):
    args = argparse.Namespace(
        ros1_bag=tmp_path / 'canonical.bag',
        ros2_bag=tmp_path / 'canonical_ros2',
        reference=tmp_path / 'gt.tum', reference_meta=tmp_path / 'ref.json',
        profile=tmp_path / 'profile.yaml', manifest=tmp_path / 'input.json',
        candidate_manifest=tmp_path / 'candidate.json',
        fast_asset_root=tmp_path / 'fast', output=tmp_path / 'out',
        glim_image='glim:frozen', fast_image='fast:frozen')
    stages = {row['name']: row['command']
              for row in MODULE.commands(args, 1.25)}
    assert set(stages) == {
        'glim_rival', 'ours_lio', 'fast_rival_baseline', 'ours_liv',
        'fast_rival_processing_probe'}
    assert '--camera-topic' not in stages['ours_lio']
    assert stages['ours_lio'][stages['ours_lio'].index('--rko-param') + 1].endswith(
        'rko_lio_ntu_viral.yaml')
    assert stages['ours_liv'][stages['ours_liv'].index('--camera-topic') + 1] == (
        '/left/image_raw')
    assert all(command[command.index('--runs') + 1] == '3'
               for command in stages.values())
    assert '--save-maps' in stages['glim_rival']
    assert '--save-maps' in stages['ours_lio']
    assert '--save-map' in stages['fast_rival_baseline']
    assert stages['glim_rival'][stages['glim_rival'].index('--image') + 1] == (
        'glim:frozen')
    assert stages['fast_rival_baseline'][
        stages['fast_rival_baseline'].index('--image') + 1] == 'fast:frozen'
    assert stages['fast_rival_processing_probe'][
        stages['fast_rival_processing_probe'].index('--rate') + 1] == '1.25'


def test_container_manifest_requires_frozen_image_ids(tmp_path, monkeypatch):
    manifest = tmp_path / 'containers.json'
    manifest.write_text(json.dumps({
        'glim_cpu': {'image': 'glim:frozen', 'image_id': 'sha256:glim'},
        'fast_livo2': {'image': 'fast:frozen', 'image_id': 'sha256:fast'},
    }))
    observed = {
        'glim:frozen': 'sha256:glim', 'fast:frozen': 'sha256:fast'}
    monkeypatch.setattr(MODULE, 'docker_image_id', observed.__getitem__)

    assert MODULE.validate_container_images(manifest) == json.loads(
        manifest.read_text())

    observed['fast:frozen'] = 'sha256:changed'
    try:
        MODULE.validate_container_images(manifest)
    except ValueError as error:
        assert 'frozen container image mismatch for fast_livo2' in str(error)
    else:
        raise AssertionError('changed image ID must be rejected')
