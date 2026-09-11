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

"""Tests for realtime coloured-map packed RGB reporting."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'evaluate_realtime_colored_map',
    REPO_ROOT / 'scripts' / 'evaluate_realtime_colored_map.py')
evaluator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluator)


def test_decode_packed_rgb_float_and_integer():
    packed = np.array([0x00112233, 0x00A0B0C0], dtype=np.uint32)
    expected = np.array([[0x11, 0x22, 0x33], [0xA0, 0xB0, 0xC0]])
    np.testing.assert_array_equal(evaluator.decode_packed_rgb(packed), expected)
    np.testing.assert_array_equal(
        evaluator.decode_packed_rgb(packed.view(np.float32)), expected)


def test_summarize_excludes_unconfirmed_grey():
    report = evaluator.summarize(np.array([
        [128, 128, 128], [200, 20, 20], [10, 30, 50]], dtype=np.uint8))
    assert report['points'] == 3
    assert report['confirmed'] == 2
    assert report['coverage'] == pytest.approx(2.0 / 3.0)
    assert report['chroma_mean'] == pytest.approx(110.0)
