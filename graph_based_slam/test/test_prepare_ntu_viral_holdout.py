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

import hashlib
import importlib.util
from pathlib import Path
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'prepare_ntu', ROOT / 'scripts' / 'prepare_ntu_viral_holdout.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_archive_verification_and_safe_extraction(tmp_path):
    archive = tmp_path / 'fixture.zip'
    with zipfile.ZipFile(archive, 'w') as stream:
        stream.writestr('fixture/sequence.bag', b'bag')
    record = MODULE.verify_archive(
        archive, archive.stat().st_size,
        hashlib.md5(archive.read_bytes()).hexdigest())
    destination = tmp_path / 'extracted'
    MODULE.safe_extract_zip(archive, destination)
    assert record['size_bytes'] == archive.stat().st_size
    assert MODULE.require_single_bag(destination).read_bytes() == b'bag'


def test_archive_size_mismatch_is_rejected(tmp_path):
    archive = tmp_path / 'fixture.zip'
    archive.write_bytes(b'not a zip')
    with pytest.raises(ValueError, match='size mismatch'):
        MODULE.verify_archive(archive, 999, 'unused')


def test_zip_path_traversal_is_rejected(tmp_path):
    archive = tmp_path / 'fixture.zip'
    with zipfile.ZipFile(archive, 'w') as stream:
        stream.writestr('../escape.bag', b'bad')
    with pytest.raises(ValueError, match='unsafe ZIP member'):
        MODULE.safe_extract_zip(archive, tmp_path / 'extracted')


def test_tree_digest_includes_relative_paths_and_contents(tmp_path):
    left = tmp_path / 'left'
    right = tmp_path / 'right'
    left.mkdir()
    right.mkdir()
    (left / 'a').write_bytes(b'value')
    (right / 'b').write_bytes(b'value')
    assert MODULE.tree_digest(left) != MODULE.tree_digest(right)
