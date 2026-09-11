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

"""Tests for isolated rosbag2 FILE-compressed playback staging."""

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'prepare_rosbag2_playback.py'


def _write_metadata(
    bag: Path,
    *,
    compression_mode: str,
    relative_path: str,
) -> bytes:
    metadata = (
        'rosbag2_bagfile_information:\n'
        '  version: 5\n'
        '  storage_identifier: sqlite3\n'
        f'  compression_format: zstd\n'
        f'  compression_mode: {compression_mode}\n'
        '  relative_file_paths:\n'
        f'    - {relative_path}\n'
    ).encode()
    (bag / 'metadata.yaml').write_bytes(metadata)
    return metadata


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['python3', str(SCRIPT_PATH), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def test_uncompressed_bag_passes_through_without_staging(tmp_path):
    """An ordinary bag should be passed to ros2 without a private view."""
    bag = tmp_path / 'bag'
    staging_root = tmp_path / 'staging'
    bag.mkdir()
    staging_root.mkdir()
    _write_metadata(
        bag,
        compression_mode='',
        relative_path='input.db3',
    )
    (bag / 'input.db3').write_bytes(b'sqlite')

    result = _run(
        'stage', '--bag', str(bag), '--staging-root', str(staging_root),
    )

    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == bag.resolve()
    assert list(staging_root.iterdir()) == []


def test_file_compressed_bag_is_staged_and_cleaned_without_touching_source(
    tmp_path,
):
    """FILE decompression should stay in a removable staging directory."""
    bag = tmp_path / 'bag'
    staging_root = tmp_path / 'staging'
    bag.mkdir()
    staging_root.mkdir()
    metadata = _write_metadata(
        bag,
        compression_mode='FILE',
        relative_path='nested/input.db3.zstd',
    )
    storage = bag / 'nested' / 'input.db3.zstd'
    storage.parent.mkdir()
    storage.write_bytes(b'compressed-storage')

    stage_result = _run(
        'stage', '--bag', str(bag), '--staging-root', str(staging_root),
    )

    assert stage_result.returncode == 0, stage_result.stderr
    staged = Path(stage_result.stdout.strip())
    staged_storage = staged / 'nested' / 'input.db3.zstd'
    assert staged.parent == staging_root.resolve()
    assert (staged / 'metadata.yaml').read_bytes() == metadata
    assert staged_storage.is_symlink()
    assert staged_storage.resolve() == storage.resolve()
    assert not (bag / 'nested' / 'input.db3').exists()

    # Model the uncompressed file that ros2 bag play creates beside the
    # staged compressed link.
    (staged / 'nested' / 'input.db3').write_bytes(b'decompressed-storage')
    cleanup_result = _run(
        'cleanup', '--path', str(staged),
        '--staging-root', str(staging_root),
    )

    assert cleanup_result.returncode == 0, cleanup_result.stderr
    assert not staged.exists()
    assert storage.read_bytes() == b'compressed-storage'
    assert not (bag / 'nested' / 'input.db3').exists()


def test_stage_rejects_storage_path_escape(tmp_path):
    """Metadata cannot make staging links escape the source bag."""
    bag = tmp_path / 'bag'
    staging_root = tmp_path / 'staging'
    bag.mkdir()
    staging_root.mkdir()
    _write_metadata(
        bag,
        compression_mode='FILE',
        relative_path='../outside.db3.zstd',
    )
    (tmp_path / 'outside.db3.zstd').write_bytes(b'outside')

    result = _run(
        'stage', '--bag', str(bag), '--staging-root', str(staging_root),
    )

    assert result.returncode == 1
    assert 'unsafe rosbag2 storage path' in result.stderr
    assert list(staging_root.iterdir()) == []


def test_cleanup_rejects_unmarked_directory(tmp_path):
    """Cleanup must fail closed for directories not created by this tool."""
    staging_root = tmp_path / 'staging'
    staged = staging_root / '.lidarslam-rosbag2-playback-unmarked'
    staged.mkdir(parents=True)
    keep = staged / 'keep.txt'
    keep.write_text('keep', encoding='utf-8')

    result = _run(
        'cleanup', '--path', str(staged),
        '--staging-root', str(staging_root),
    )

    assert result.returncode == 1
    assert 'staging marker is missing or unsafe' in result.stderr
    assert keep.read_text(encoding='utf-8') == 'keep'
