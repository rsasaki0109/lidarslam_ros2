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

"""Tests for the Docker runtime apt-closure collector."""

import importlib.util
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'collect_runtime_apt_packages',
    ROOT / 'docker' / 'collect_runtime_apt_packages.py',
)
COLLECTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COLLECTOR)


def _result(returncode=0, stdout='', stderr=''):
    return subprocess.CompletedProcess([], returncode, stdout, stderr)


def test_parse_ldd_output_accepts_both_dependency_shapes():
    """Both normal ldd dependency forms resolve to absolute paths."""
    output = """
linux-vdso.so.1 (0x00007fff)
libfoo.so.1 => /usr/lib/x86_64-linux-gnu/libfoo.so.1 (0x00001)
/lib64/ld-linux-x86-64.so.2 (0x00002)
"""

    assert COLLECTOR.parse_ldd_output(output, 'fixture') == {
        Path('/usr/lib/x86_64-linux-gnu/libfoo.so.1'),
        Path('/lib64/ld-linux-x86-64.so.2'),
    }


def test_parse_ldd_output_rejects_missing_library():
    """An unresolved shared object makes closure derivation fail closed."""
    with pytest.raises(COLLECTOR.RuntimePackageError, match='libmissing.so'):
        COLLECTOR.parse_ldd_output(
            'libmissing.so => not found\n', 'fixture'
        )


def test_parse_owner_output_preserves_architecture_and_rejects_injection():
    """Architecture qualifiers survive while unsafe owner text is rejected."""
    assert COLLECTOR.parse_owner_output(
        'diversion by libc6 from:amd64 from: /lib64/ld-linux.so.2\n'
        'libc6:amd64: /usr/lib/x86_64-linux-gnu/libc.so.6\n'
        'ros-jazzy-rclcpp: /opt/ros/jazzy/lib/librclcpp.so\n'
    ) == {'libc6:amd64', 'ros-jazzy-rclcpp'}

    with pytest.raises(COLLECTOR.RuntimePackageError, match='unsafe'):
        COLLECTOR.parse_owner_output('bad package: /tmp/file\n')


def test_package_owners_retries_resolved_merged_usr_path(tmp_path):
    """Merged-/usr aliases retry their canonical package-owned paths."""
    target = tmp_path / 'real'
    target.write_bytes(b'fixture')
    alias = tmp_path / 'alias'
    alias.symlink_to(target)
    calls = []

    def runner(arguments):
        calls.append(tuple(arguments))
        if arguments[-1] == str(target):
            return _result(stdout='fixture-runtime: ' + str(target) + '\n')
        return _result(returncode=1, stderr='not owned')

    assert COLLECTOR.package_owners(alias, runner) == {'fixture-runtime'}
    assert calls == [
        ('dpkg-query', '-S', str(alias)),
        ('dpkg-query', '-S', str(target)),
    ]


def test_package_owners_retries_verified_reverse_merged_usr_alias(
    tmp_path, monkeypatch
):
    """Legacy dpkg paths are tried only for a verified merged-/usr pair."""
    usr_bin = tmp_path / 'usr-bin'
    usr_bin.mkdir()
    executable = usr_bin / 'bash'
    executable.write_bytes(b'fixture')
    legacy_bin = tmp_path / 'bin'
    legacy_bin.symlink_to(usr_bin, target_is_directory=True)
    monkeypatch.setattr(
        COLLECTOR,
        'MERGED_USR_DIRECTORY_PAIRS',
        ((legacy_bin, usr_bin),),
    )
    calls = []

    def runner(arguments):
        calls.append(tuple(arguments))
        if arguments[-1] == str(legacy_bin / 'bash'):
            return _result(stdout='bash: ' + str(legacy_bin / 'bash') + '\n')
        return _result(returncode=1, stderr='not owned')

    assert COLLECTOR.package_owners(executable, runner) == {'bash'}
    assert calls == [
        ('dpkg-query', '-S', str(executable)),
        ('dpkg-query', '-S', str(legacy_bin / 'bash')),
    ]


def test_reverse_merged_usr_alias_requires_same_resolved_directory(tmp_path):
    """Unrelated legacy and /usr roots cannot create owner candidates."""
    usr_bin = tmp_path / 'usr-bin'
    usr_bin.mkdir()
    executable = usr_bin / 'bash'
    executable.write_bytes(b'fixture')
    unrelated_bin = tmp_path / 'unrelated-bin'
    unrelated_bin.mkdir()
    (unrelated_bin / 'bash').write_bytes(b'other')

    assert COLLECTOR.owner_path_candidates(
        executable,
        ((unrelated_bin, usr_bin),),
    ) == [executable]


def test_linked_libraries_rejects_unexpected_ldd_failure(tmp_path):
    """Unexpected ldd errors cannot be mistaken for static executables."""
    binary = tmp_path / 'binary'
    binary.write_bytes(COLLECTOR.ELF_MAGIC)

    def runner(_arguments):
        return _result(returncode=1, stderr='permission denied')

    with pytest.raises(COLLECTOR.RuntimePackageError, match='ldd failed'):
        COLLECTOR.linked_libraries([binary], runner)


def test_explicit_package_validation_rejects_unsafe_name():
    """Explicit package values must obey the Debian package-name grammar."""
    with pytest.raises(COLLECTOR.RuntimePackageError, match='unsafe'):
        COLLECTOR._installed_package('$(touch bad)', lambda _: _result())


def test_runtime_contract_covers_discovered_cli_plugins_and_shell_tools():
    """Non-ELF plugin and shell requirements remain explicitly covered."""
    assert COLLECTOR.required_ros_packages('jazzy') == (
        'ros-jazzy-ros2launch',
        'ros-jazzy-rosbag2',
        'ros-jazzy-rosbag2-storage-mcap',
    )
    assert {
        'awk',
        'realpath',
        'setsid',
        'stat',
        'tar',
    }.issubset(COLLECTOR.DEFAULT_COMMANDS)
