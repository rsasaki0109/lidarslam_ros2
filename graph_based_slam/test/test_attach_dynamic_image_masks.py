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

"""Tests for dynamic-image-mask manifest attachment."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import imageio as iio
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import attach_dynamic_image_masks as adm  # noqa: E402, I100


def _dataset(root: Path, frames: int = 2) -> Path:
    images = root / 'images'
    images.mkdir(parents=True)
    entries = []
    for index in range(frames):
        iio.imwrite(images / f'{index}.png', np.zeros((6, 8, 3), np.uint8))
        entries.append({
            'file_path': f'images/{index}.png',
            'transform_matrix': np.eye(4).tolist(),
        })
    source = root / 'transforms.json'
    source.write_text(json.dumps({
        'w': 8, 'h': 6, 'fl_x': 5, 'fl_y': 5, 'cx': 4, 'cy': 3,
        'frames': entries,
    }))
    return source


def test_attach_masks_records_paths_hashes_and_coverage(tmp_path):
    source = _dataset(tmp_path / 'source')
    masks = tmp_path / 'masks'
    masks.mkdir()
    first = np.zeros((6, 8), np.uint8)
    first[1:3, 2:4] = 255
    iio.imwrite(masks / '0.png', first)
    iio.imwrite(masks / '1.png', np.zeros((6, 8), np.uint8))
    output = tmp_path / 'result' / 'transforms_masks.json'
    report = adm.attach_masks(source, masks, output)
    assert report['complete'] and report['frames_with_masks'] == 2
    assert report['masked_pixel_fraction'] == 4 / (2 * 6 * 8)
    assert len(report['sha256_by_frame_index']['0']) == 64
    document = json.loads(output.read_text())
    for frame in document['frames']:
        assert (output.parent / frame['file_path']).resolve().is_file()
        assert (output.parent / frame['dynamic_mask_path']).resolve().is_file()


def test_attach_masks_rejects_missing_and_wrong_shape(tmp_path):
    source = _dataset(tmp_path / 'source', frames=1)
    masks = tmp_path / 'masks'
    masks.mkdir()
    with np.testing.assert_raises(FileNotFoundError):
        adm.attach_masks(source, masks, tmp_path / 'missing.json')
    iio.imwrite(masks / '0.png', np.zeros((5, 8), np.uint8))
    with np.testing.assert_raises(ValueError):
        adm.attach_masks(source, masks, tmp_path / 'wrong.json')


def test_attach_masks_can_mark_partial_dataset(tmp_path):
    source = _dataset(tmp_path / 'source')
    masks = tmp_path / 'masks'
    masks.mkdir()
    iio.imwrite(masks / '0.png', np.zeros((6, 8), np.uint8))
    output = tmp_path / 'partial.json'
    report = adm.attach_masks(source, masks, output, allow_missing=True)
    assert not report['complete'] and report['frames_with_masks'] == 1
    frames = json.loads(output.read_text())['frames']
    assert 'dynamic_mask_path' in frames[0]
    assert 'dynamic_mask_path' not in frames[1]
