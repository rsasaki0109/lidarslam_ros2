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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for claim-bounded, captioned short-demo media generation."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
VIDEO_SCRIPT = SCRIPTS / 'generate_social_autoware_demo_video.py'
CARD_SCRIPT = SCRIPTS / 'generate_social_autoware_map_authoring_card.py'


def _load(path: Path, name: str):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


MEDIA = _load(VIDEO_SCRIPT, 'social_demo_media_test')


def _write_contract(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / 'contract.json'
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return path


def _generated_files(tmp_path: Path) -> Path:
    output = tmp_path / 'bundle'
    output.mkdir()
    (output / MEDIA.VIDEO_NAME).write_bytes(b'video-bytes')
    (output / MEDIA.CAPTIONS_NAME).write_text(
        MEDIA.build_captions(MEDIA.load_contract()),
        encoding='utf-8',
    )
    (output / MEDIA.POST_NAME).write_text(
        MEDIA.build_post_copy(
            MEDIA.load_contract(),
            'a' * 40,
            'PUBLICATION_CANDIDATE',
        ),
        encoding='utf-8',
    )
    return output


def test_contract_binds_current_version_commands_images_and_no_public_claims():
    contract = MEDIA.load_contract()

    assert contract['product_version'] == '0.9.1'
    assert contract['commands'] == {
        'fixed_demo': 'lidarslam-map demo',
        'own_bag': 'lidarslam-map start /path/to/rosbag2',
    }
    assert len(contract['source_images']) == 3
    assert [slide['id'] for slide in contract['slides']] == [
        'promise',
        'beginner-path',
        'own-bag',
        'proof',
    ]
    rendered = json.dumps(contract, ensure_ascii=False)
    assert all(token not in rendered for token in MEDIA.FORBIDDEN_PUBLIC_COPY)
    assert contract['publication_boundary'] == {
        'external_publication_authorized': False,
        'writes_performed': False,
    }


def test_contract_rejects_stale_release_copy(tmp_path: Path):
    contract = copy.deepcopy(MEDIA.load_contract())
    contract['slides'][0]['title'] = 'lidarslam_ros2 v0.2.2 is out'

    with pytest.raises(MEDIA.MediaError, match='stale or unsupported'):
        MEDIA.load_contract(_write_contract(tmp_path, contract))


def test_contract_rejects_command_and_source_image_drift(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    contract = copy.deepcopy(MEDIA.load_contract())
    contract['commands']['fixed_demo'] = 'python3 mystery.py'
    with pytest.raises(MEDIA.MediaError, match='schema validation'):
        MEDIA.load_contract(_write_contract(tmp_path, contract))

    original = MEDIA._regular_file

    def changed(path: Path, label: str) -> bytes:
        payload = original(path, label)
        if path == ROOT / 'lidarslam' / 'images' / 'map.png':
            return payload + b'drift'
        return payload

    monkeypatch.setattr(MEDIA, '_regular_file', changed)
    with pytest.raises(MEDIA.MediaError, match='SHA-256 drift'):
        MEDIA.load_contract()


def test_captions_cover_four_non_overlapping_cues_below_three_minutes():
    contract = MEDIA.load_contract()
    captions = MEDIA.build_captions(contract)

    assert captions.startswith('WEBVTT\n\n')
    assert captions.count(' --> ') == 4
    assert '00:00:00.000 --> 00:00:02.550' in captions
    assert '00:00:07.650 --> 00:00:10.667' in captions
    assert MEDIA._expected_duration(contract) < 180
    for slide in contract['slides']:
        assert slide['caption'] in captions


def test_slide_renderer_uses_contract_copy_at_both_output_sizes():
    contract = MEDIA.load_contract()
    first = contract['slides'][0]

    assert MEDIA.render_slide(contract, first).size == (1280, 720)
    assert MEDIA.render_slide(contract, first, size=(1600, 900)).size == (
        1600,
        900,
    )


def test_slide_renderer_accepts_legacy_pillow_resampling_api():
    class LegacyImageModule:
        LANCZOS = 1

    assert MEDIA._lanczos_resample(LegacyImageModule) == 1


def test_post_copy_is_exact_revision_bounded_and_not_a_release_claim():
    contract = MEDIA.load_contract()
    revision = 'b' * 40
    post = MEDIA.build_post_copy(
        contract,
        revision,
        'PUBLICATION_CANDIDATE',
    )

    assert 'NOT_PUBLISHED' in post
    assert 'External publication authorized: **false**' in post
    assert f'/blob/{revision}/docs/getting-started.md' in post
    assert 'lidarslam-map demo' in post
    assert 'lidarslam-map start /path/to/rosbag2' in post
    assert '性能優位' in post
    assert all(token not in post for token in MEDIA.FORBIDDEN_PUBLIC_COPY)


def test_revision_gate_distinguishes_clean_candidate_and_dirty_preview(
    monkeypatch: pytest.MonkeyPatch,
):
    revision = 'c' * 40

    def clean(args):
        return revision if args == ('rev-parse', 'HEAD') else ''

    monkeypatch.setattr(MEDIA, '_git_output', clean)
    assert MEDIA.inspect_source_revision(revision, False) == 'PUBLICATION_CANDIDATE'

    def dirty(args):
        return revision if args == ('rev-parse', 'HEAD') else ' M README.md'

    monkeypatch.setattr(MEDIA, '_git_output', dirty)
    with pytest.raises(MEDIA.MediaError, match='clean source worktree'):
        MEDIA.inspect_source_revision(revision, False)
    assert MEDIA.inspect_source_revision(revision, True) == 'LOCAL_PREVIEW'

    with pytest.raises(MEDIA.MediaError, match='does not match HEAD'):
        MEDIA.inspect_source_revision('d' * 40, True)


def test_manifest_binds_every_byte_and_denies_publication(tmp_path: Path):
    contract = MEDIA.load_contract()
    output = _generated_files(tmp_path)
    manifest = MEDIA.build_manifest(
        contract,
        'a' * 40,
        'PUBLICATION_CANDIDATE',
        output,
        {
            'codec': 'h264',
            'width': 1280,
            'height': 720,
            'fps': 24,
            'duration_seconds': 10.666667,
        },
    )

    assert manifest['video']['sha256'] == MEDIA._sha256(b'video-bytes')
    assert manifest['captions']['cue_count'] == 4
    assert manifest['claims'] == {
        'numerical_performance_claims': False,
        'release_claim': False,
        'package_availability_claim': False,
        'sensor_compatibility_claim': False,
    }
    assert manifest['publication_boundary']['external_publication_authorized'] is False

    sys.path.insert(0, str(SCRIPTS))
    try:
        from product_schema import validate_contract

        validate_contract(manifest, MEDIA.MANIFEST_SCHEMA)
    finally:
        sys.path.remove(str(SCRIPTS))


def test_checked_in_media_matches_clean_generator_revision_and_manifest():
    manifest_path = (
        ROOT
        / 'lidarslam'
        / 'images'
        / 'social_autoware_map_authoring_demo.manifest.json'
    )
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))

    sys.path.insert(0, str(SCRIPTS))
    try:
        from product_schema import validate_contract

        validate_contract(manifest, MEDIA.MANIFEST_SCHEMA)
    finally:
        sys.path.remove(str(SCRIPTS))

    assert manifest['status'] == 'PUBLICATION_CANDIDATE'
    source_revision = manifest['source_revision']
    assert subprocess.run(
        ['git', 'merge-base', '--is-ancestor', source_revision, 'HEAD'],
        cwd=ROOT,
        check=False,
    ).returncode == 0
    contract_at_revision = subprocess.run(
        [
            'git',
            'show',
            f'{source_revision}:docs/contracts/social-demo-media-v1.json',
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
    ).stdout
    assert MEDIA._sha256(contract_at_revision) == manifest['content_contract']['sha256']

    locations = {
        'video': ROOT / 'lidarslam' / 'images' / MEDIA.VIDEO_NAME,
        'captions': ROOT / 'lidarslam' / 'images' / MEDIA.CAPTIONS_NAME,
        'post_copy': ROOT / 'docs' / 'social' / MEDIA.POST_NAME,
    }
    for key, path in locations.items():
        payload = path.read_bytes()
        assert len(payload) == manifest[key]['size_bytes']
        assert MEDIA._sha256(payload) == manifest[key]['sha256']

    assert manifest['publication_boundary'] == {
        'external_publication_authorized': False,
        'writes_performed': False,
    }


def test_output_is_write_once_and_ffmpeg_command_is_deterministic(tmp_path: Path):
    contract = MEDIA.load_contract()
    output = tmp_path / 'existing'
    output.mkdir()
    with pytest.raises(MEDIA.MediaError, match='refusing to overwrite'):
        MEDIA.generate_bundle(
            contract,
            output,
            'a' * 40,
            'LOCAL_PREVIEW',
        )

    slides = [tmp_path / f'{index}.png' for index in range(4)]
    command = MEDIA._ffmpeg_cmd(contract, slides, tmp_path / 'video.mp4')
    assert '-y' not in command
    assert command[command.index('-threads') + 1] == '1'
    assert '+bitexact' in command
    assert command[command.index('-map_metadata') + 1] == '-1'


def test_card_generator_reuses_the_same_contract(tmp_path: Path):
    card = _load(CARD_SCRIPT, 'social_demo_card_test')
    output = tmp_path / 'card.png'

    assert card.main(['--out', str(output), '--slide', 'promise']) == 0
    assert output.is_file()
    with MEDIA.Image.open(output) as image:
        assert image.size == (1600, 900)
