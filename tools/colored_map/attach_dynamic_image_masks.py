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

"""Attach externally generated dynamic-object masks to a transforms dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


def attach_masks(source: Path, mask_dir: Path, output: Path, *,
                 allow_missing: bool = False) -> dict:
    """Validate masks, rewrite relative paths, and return provenance summary."""
    import imageio as iio

    source, mask_dir, output = (
        Path(source).resolve(), Path(mask_dir).resolve(), Path(output).resolve())
    if source == output:
        raise ValueError('dynamic-mask output must differ from source transforms')
    document = json.loads(source.read_text())
    height, width = int(document['h']), int(document['w'])
    attached = 0
    masked_pixels = 0
    hashes = {}
    for index, frame in enumerate(document['frames']):
        image_path = (source.parent / frame['file_path']).resolve()
        candidate = mask_dir / (Path(frame['file_path']).stem + '.png')
        frame['file_path'] = os.path.relpath(image_path, output.parent)
        if not candidate.is_file():
            if allow_missing:
                frame.pop('dynamic_mask_path', None)
                continue
            raise FileNotFoundError(
                f'missing dynamic mask for frame {index}: {candidate}')
        mask = np.asarray(iio.imread(candidate))
        if mask.shape[:2] != (height, width):
            raise ValueError(
                f'dynamic mask {candidate} has shape {mask.shape[:2]}, '
                f'expected {(height, width)}')
        if mask.ndim == 3:
            mask = np.any(mask != 0, axis=2)
        elif mask.ndim == 2:
            mask = mask != 0
        else:
            raise ValueError(f'dynamic mask must be HxW or HxWxC: {candidate}')
        frame['dynamic_mask_path'] = os.path.relpath(candidate, output.parent)
        attached += 1
        masked_pixels += int(mask.sum())
        hashes[str(index)] = hashlib.sha256(candidate.read_bytes()).hexdigest()
    frames = len(document['frames'])
    provenance = {
        'schema': 'dynamic-image-mask-v1',
        'frames_total': frames,
        'frames_with_masks': attached,
        'complete': attached == frames,
        'masked_pixel_fraction': (
            masked_pixels / (frames * height * width) if frames else 0.0),
        'sha256_by_frame_index': hashes,
    }
    document['dynamic_mask_provenance'] = provenance
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return provenance


def build_parser() -> argparse.ArgumentParser:
    """Construct the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--mask-dir', type=Path, required=True,
                        help='PNG masks named after each image stem')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--allow-missing', action='store_true')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    print(json.dumps(attach_masks(
        args.transforms, args.mask_dir, args.out,
        allow_missing=args.allow_missing), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
