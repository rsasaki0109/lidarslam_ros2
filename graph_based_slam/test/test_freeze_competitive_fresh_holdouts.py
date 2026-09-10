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
Safety tests for the networked fresh-holdout freezer.

The fixture HTTP server is local and serves only tiny synthetic artifacts.  No
repository or benchmark data is contacted by these tests.
"""

from __future__ import annotations

from functools import partial
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import subprocess
import threading
from urllib.parse import urlparse

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'freeze_competitive_fresh_holdouts.py'
SPEC = importlib.util.spec_from_file_location('fresh_holdout_freezer', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class _RangeHandler(BaseHTTPRequestHandler):
    root: Path

    def do_GET(self):  # noqa: N802 - stdlib handler API
        path = self.root / Path(urlparse(self.path).path).name
        if not path.is_file():
            self.send_error(404)
            return
        payload = path.read_bytes()
        start = 0
        requested = self.headers.get('Range')
        if requested and requested.startswith('bytes='):
            start = int(requested[6:].split('-', 1)[0])
            if start >= len(payload):
                self.send_error(416)
                return
        self.send_response(206 if start else 200)
        self.send_header('Content-Length', str(len(payload) - start))
        if start:
            self.send_header('Content-Range',
                             f'bytes {start}-{len(payload) - 1}/{len(payload)}')
        self.end_headers()
        self.wfile.write(payload[start:])

    def log_message(self, _format, *_args):
        return


def _serve(root: Path):
    class Handler(_RangeHandler):
        pass

    Handler.root = root
    handler = Handler
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _fixture(tmp_path: Path):
    calibration_names = ['calib-a.yaml', 'calib-b.yaml']
    for name, payload in (
            ('calib-a.yaml', b'calibration-a'),
            ('calib-b.yaml', b'calibration-b')):
        (tmp_path / name).write_bytes(payload)
    candidates = {}
    for index, sequence in enumerate(('exp14', 'exp16', 'exp18'), start=1):
        bag_name = f'{sequence}.bag'
        gt_name = f'{sequence}_imu.txt'
        bag = f'bag-{index}'.encode()
        gt = f'SECRET-GT-{index}'.encode()
        (tmp_path / bag_name).write_bytes(bag)
        (tmp_path / gt_name).write_bytes(gt)
        candidates[f'fresh_{index}'] = {
            'dataset': f'fixture_{sequence}',
            'sequence': sequence,
            'bag': {
                'url': f'https://huggingface.co/datasets/example/repo/blob/'
                       f'e62017f907007fdc5ab8c721842e4ae7359d7f49/{bag_name}',
                'expected_bytes': len(bag),
                'lfs_sha256': hashlib.sha256(bag).hexdigest(),
            },
            'ground_truth': {
                'url': f'https://huggingface.co/datasets/example/repo/blob/'
                       f'e62017f907007fdc5ab8c721842e4ae7359d7f49/{gt_name}',
                'expected_bytes': len(gt),
                'git_blob_oid': MODULE.git_blob_sha1(tmp_path / gt_name),
            },
        }
    relevant = {}
    for name in calibration_names:
        path = tmp_path / name
        relevant[name] = {
            'expected_bytes': path.stat().st_size,
            'git_blob_oid': MODULE.git_blob_sha1(path),
        }
    selection = {
        'receipt_kind': 'm5b_fresh_holdout_selection',
        'official_source': {
            'repository': 'https://huggingface.co/datasets/example/repo',
            'revision': 'e62017f907007fdc5ab8c721842e4ae7359d7f49',
            'license': 'cc-by-nc-sa-3.0',
            'calibration': {
                'tree_path': 'calibration/calibration_files',
                'tree_oid': 'a' * 40,
                'relevant_files': relevant,
            },
        },
        'selection_decision': {'selected_candidates': candidates},
    }
    selection_path = tmp_path / 'selection.yaml'
    selection_path.write_text(yaml.safe_dump(selection), encoding='utf-8')
    return selection_path, candidates


def _local_curl(server, url, part, resume):
    filename = Path(urlparse(url).path).name
    local_url = f'http://127.0.0.1:{server.server_port}/{filename}'
    command = ['curl', '--fail', '--silent', '--show-error']
    if resume:
        command.extend(['--continue-at', '-'])
    command.extend(['--output', str(part), local_url])
    subprocess.run(command, check=True)


def test_plan_is_dry_run_and_uses_pinned_resolve_urls(tmp_path, capsys):
    selection, _ = _fixture(tmp_path)
    destination = tmp_path / 'not-created'
    plan = MODULE.build_plan(selection, destination)
    assert not destination.exists()
    assert len(plan['slots']) == 3
    assert plan['producer']['path'] == 'scripts/freeze_competitive_fresh_holdouts.py'
    assert plan['producer']['sha256'] == MODULE.sha256_file(SCRIPT)
    assert all('/resolve/e62017f907007fdc5ab8c721842e4ae7359d7f49/' in
               slot['artifacts'][0]['url'] for slot in plan['slots'])
    MODULE.main(['plan', '--selection', str(selection), '--root', str(destination)])
    assert 'SECRET-GT' not in capsys.readouterr().out


def test_local_http_resume_hashes_without_logging_ground_truth(tmp_path, monkeypatch,
                                                               capsys):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    server, thread = _serve(tmp_path)
    monkeypatch.setattr(MODULE, '_run_curl',
                        partial(_local_curl, server))
    try:
        slot = plan['slots'][0]
        root = tmp_path / 'destination'
        MODULE._ensure_managed_root(root, plan, create=True)
        stage = root / slot['paths']['stage']
        stage.mkdir(parents=True)
        state_path = root / '.state' / 'exp14.json'
        state_path.parent.mkdir(parents=True)
        state_path.write_text(json.dumps({
            'plan_sha256': slot['plan_sha256'], 'status': 'downloading',
            'sequence': 'exp14', 'artifacts': {},
        }), encoding='utf-8')
        raw = slot['artifacts'][0]
        partial_path = stage / raw['stage_relative_path']
        partial_path.parent.mkdir(parents=True)
        partial_path.with_name(partial_path.name + '.part').write_bytes(b'ba')
        manifest = MODULE._download_slot(root, slot, resume=True)
        output = capsys.readouterr().out
        assert 'SECRET-GT' not in output
        document = json.loads(manifest.read_text(encoding='utf-8'))
        assert document['status'] == 'downloaded_hashed'
        assert document['source']['plan_sha256'] == plan['plan_sha256']
        assert document['ground_truth']['content_opened'] is False
        assert document['ground_truth']['sha256'] == hashlib.sha256(
            b'SECRET-GT-1').hexdigest()
        calibration_file = document['calibration']['files'][0]
        assert calibration_file['path'].startswith(
            'slots/exp14/calibration/files/')
        assert calibration_file['logical_path'].startswith(
            'calibration/calibration_files/')
        assert MODULE._verify_manifest(manifest, root) == 'downloaded_hashed'
        marker = json.loads((root / '.freeze-root.json').read_text(encoding='utf-8'))
        marker['plan_sha256'] = '0' * 64
        (root / '.freeze-root.json').write_text(
            json.dumps(marker), encoding='utf-8')
        with pytest.raises(ValueError, match='plan identity'):
            MODULE._verify_manifest(manifest, root)
        assert manifest.is_file()
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_complete_final_slot_is_verified_and_skipped_on_resume(tmp_path, monkeypatch):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    server, thread = _serve(tmp_path)
    monkeypatch.setattr(MODULE, '_run_curl', partial(_local_curl, server))
    try:
        root = tmp_path / 'destination'
        slot = plan['slots'][0]
        manifest = MODULE._download_slot(root, slot, resume=False)
        assert MODULE._download_slot(root, slot, resume=True) == manifest
        stage = root / slot['paths']['stage']
        stage.mkdir(parents=True)
        with pytest.raises(ValueError, match='coexist'):
            MODULE._download_slot(root, slot, resume=True)
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_complete_part_is_verified_and_renamed_before_resume(tmp_path, monkeypatch):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    server, thread = _serve(tmp_path)
    monkeypatch.setattr(MODULE, '_run_curl', partial(_local_curl, server))
    try:
        slot = plan['slots'][0]
        root = tmp_path / 'destination'
        stage = root / slot['paths']['stage']
        stage.mkdir(parents=True)
        state_path = root / '.state' / 'exp14.json'
        state_path.parent.mkdir(parents=True)
        state_path.write_text(json.dumps({
            'plan_sha256': slot['plan_sha256'], 'status': 'downloading',
            'sequence': 'exp14', 'artifacts': {},
        }), encoding='utf-8')
        raw = slot['artifacts'][0]
        part = stage / raw['stage_relative_path']
        part.parent.mkdir(parents=True)
        part.with_name(part.name + '.part').write_bytes(b'bag-1')
        manifest = MODULE._download_slot(root, slot, resume=True)
        assert (stage / raw['stage_relative_path']).exists() is False
        assert manifest.is_file()
        assert not list(root.rglob('*.part'))
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_full_part_hash_mismatch_is_not_restarted(tmp_path, monkeypatch):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    slot = plan['slots'][0]
    root = tmp_path / 'destination'
    stage = root / slot['paths']['stage']
    stage.mkdir(parents=True)
    state_path = root / '.state' / 'exp14.json'
    state_path.parent.mkdir(parents=True)
    state_path.write_text(json.dumps({
        'plan_sha256': slot['plan_sha256'], 'status': 'downloading',
        'sequence': 'exp14', 'artifacts': {},
    }), encoding='utf-8')
    raw = slot['artifacts'][0]
    partial_path = stage / raw['stage_relative_path']
    partial_path.parent.mkdir(parents=True)
    partial_path.with_name(partial_path.name + '.part').write_bytes(b'xxxxx')
    called = False

    def unexpected_curl(*_args, **_kwargs):
        nonlocal called
        called = True

    monkeypatch.setattr(MODULE, '_run_curl', unexpected_curl)
    with pytest.raises(ValueError, match='SHA-256'):
        MODULE._download_slot(root, slot, resume=True)
    assert called is False


def test_finalize_requires_verified_files_and_accepts_clean_manifest(tmp_path,
                                                                     monkeypatch):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    server, thread = _serve(tmp_path)
    monkeypatch.setattr(MODULE, '_run_curl', partial(_local_curl, server))
    try:
        root = tmp_path / 'destination'
        slot = plan['slots'][0]
        manifest_path = MODULE._download_slot(root, slot, resume=False)
        ros2_root = tmp_path / 'ros2'
        ros2_root.mkdir()
        metadata = {
            'rosbag2_bagfile_information': {
                'topics_with_message_count': [
                    {'topic_metadata': {'name': name, 'type': msg_type}}
                    for name, msg_type, _role in MODULE.CANONICAL_TOPIC_CONTRACT],
            },
        }
        (ros2_root / 'metadata.yaml').write_text(
            yaml.safe_dump(metadata), encoding='utf-8')
        (ros2_root / 'bag.db3').write_bytes(b'ros2')
        semantic = {
            'all_topics_equal': True,
            'topics': [{
                'topic': name,
                'equal': True,
                'message_count_left': 1,
                'message_count_right': 1,
                'aggregate_sha256_left': '0' * 64,
                'aggregate_sha256_right': '0' * 64,
            } for name, _msg_type, _role in MODULE.CANONICAL_TOPIC_CONTRACT],
        }
        semantic_path = tmp_path / 'semantic.json'
        semantic_path.write_text(json.dumps(semantic), encoding='utf-8')
        finalized = MODULE._finalize_manifest(
            manifest_path, root, ros2_root, semantic_path)
        assert finalized['status'] == 'frozen_unopened'
        assert len(finalized['input_manifest_sha256']) == 64

        document = json.loads(manifest_path.read_text(encoding='utf-8'))
        calibration_path = root / document['calibration']['files'][0]['path']
        calibration_path.write_bytes(b'corrupted-a')
        with pytest.raises(ValueError, match='calibration'):
            MODULE._finalize_manifest(
                manifest_path, root, ros2_root, semantic_path)
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_existing_wrong_file_and_hash_mismatch_fail_closed(tmp_path):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    artifact = plan['slots'][0]['artifacts'][0]
    path = tmp_path / 'wrong'
    path.write_bytes(b'wrong')
    bad = dict(artifact)
    bad['expected_bytes'] = len(b'expected')
    with pytest.raises(ValueError, match='byte count mismatch'):
        MODULE._verify_file(path, bad)

    bad['expected_bytes'] = len(b'wrong')
    bad['expected_sha256'] = '0' * 64
    with pytest.raises(ValueError, match='SHA-256'):
        MODULE._verify_file(path, bad)


def test_path_traversal_and_bad_source_revision_are_rejected(tmp_path):
    with pytest.raises(ValueError, match='must not contain'):
        MODULE._require_relative_path('../escape', 'fixture')
    selection, _ = _fixture(tmp_path)
    document = yaml.safe_load(selection.read_text(encoding='utf-8'))
    document['official_source']['revision'] = 'not-a-commit'
    selection.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(ValueError, match='full 40-hex commit'):
        MODULE.build_plan(selection, tmp_path / 'destination')


def test_manifest_json_is_canonical_and_atomic(tmp_path):
    path = tmp_path / 'manifest.json'
    MODULE._atomic_json(path, {'z': 1, 'a': {'b': 2}})
    assert path.read_text(encoding='utf-8') == '{"a":{"b":2},"z":1}\n'
    assert json.loads(path.read_text(encoding='utf-8')) == {'a': {'b': 2}, 'z': 1}
    assert not list(tmp_path.glob('.manifest.json.*.tmp'))


def test_existing_unmanaged_destination_is_rejected(tmp_path):
    selection, _ = _fixture(tmp_path)
    plan = MODULE.build_plan(selection, tmp_path / 'destination')
    destination = tmp_path / 'destination'
    destination.mkdir()
    (destination / 'foreign.txt').write_text('unmanaged', encoding='utf-8')
    with pytest.raises(ValueError, match='untracked files'):
        MODULE._ensure_managed_root(destination, plan, create=True)
