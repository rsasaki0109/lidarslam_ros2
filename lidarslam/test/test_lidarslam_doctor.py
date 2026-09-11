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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Tests for bag-optional, read-only product installation diagnosis."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'lidarslam_doctor.py'
SCHEMA_PATH = ROOT / 'docs' / 'schemas' / 'system-doctor-v1.schema.json'
CLI = ROOT / 'scripts' / 'lidarslam'
SPEC = importlib.util.spec_from_file_location('lidarslam_doctor_test', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
DOCTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DOCTOR)


def _layout(
    tmp_path: Path,
    *,
    source: bool,
    runtime_files: tuple[str, ...] = (
        'lidarslam_doctor.py',
        'preflight_autoware_map_bag.py',
    ),
) -> Path:
    if source:
        root = tmp_path / 'checkout'
        script_dir = root / 'scripts'
        (root / 'lidarslam').mkdir(parents=True)
        (root / 'lidarslam' / 'package.xml').write_text(
            '<package/>\n', encoding='utf-8'
        )
        manifest = root / 'lidarslam' / 'product-runtime-files.txt'
    else:
        product = tmp_path / 'prefix' / 'share' / 'lidarslam' / 'product'
        script_dir = product / 'scripts'
        manifest = product / 'product-runtime-files.txt'
        root = product
    script_dir.mkdir(parents=True)
    (root / 'VERSION').write_text('0.9.0\n', encoding='utf-8')
    manifest.write_text('\n'.join(runtime_files) + '\n', encoding='utf-8')
    for name in runtime_files:
        (script_dir / name).write_text('# fixture\n', encoding='utf-8')
    return script_dir


def _disk(free_gib: float):
    return lambda _path: SimpleNamespace(free=int(free_gib * DOCTOR.GIB))


def _ready_report(tmp_path: Path, *, source: bool) -> dict:
    script_dir = _layout(tmp_path, source=source)
    environment = {'ROS_DISTRO': 'jazzy'}
    if source:
        prefix = tmp_path / 'ready-prefix'
        marker = prefix / 'share' / 'lidarslam' / 'product' / 'VERSION'
        marker.parent.mkdir(parents=True)
        marker.write_text('0.9.0\n', encoding='utf-8')
        environment['AMENT_PREFIX_PATH'] = str(prefix)
    return DOCTOR.build_system_report(
        script_dir=script_dir,
        demo_dir=tmp_path,
        min_free_space_gib=8,
        environment=environment,
        command_lookup=lambda name: '/usr/bin/ros2' if name == 'ros2' else None,
        module_available=lambda name: name == 'rosbag2_py',
        disk_usage=_disk(12),
    )


@pytest.mark.parametrize('source', [True, False])
def test_ready_source_and_installed_layouts_are_schema_valid(tmp_path, source):
    report = _ready_report(tmp_path, source=source)

    jsonschema.validate(
        report,
        json.loads(SCHEMA_PATH.read_text(encoding='utf-8')),
    )
    assert report['status'] == 'ready'
    assert report['findings'] == []
    assert report['next_action'] is None
    assert report['product']['layout'] == ('source' if source else 'installed')
    assert report['product']['installed_prefix_detected'] is True
    assert report['storage']['additional_bytes_required'] == 0
    assert report['network_accessed'] is False
    assert report['writes_performed'] is False


def test_source_without_build_reports_one_copy_ready_recovery(tmp_path):
    script_dir = _layout(tmp_path, source=True)

    report = DOCTOR.build_system_report(
        script_dir=script_dir,
        demo_dir=tmp_path,
        min_free_space_gib=8,
        environment={'ROS_DISTRO': 'humble'},
        command_lookup=lambda _name: '/opt/ros/humble/bin/ros2',
        module_available=lambda _name: True,
        disk_usage=_disk(10),
    )

    assert report['status'] == 'action_required'
    assert [item['code'] for item in report['findings']] == [
        'source-build-required'
    ]
    assert 'source_quickstart.sh --build-only' in (
        report['findings'][0]['next_action']
    )
    assert report['next_action'] == {
        'code': 'source-build-required',
        'reason': report['findings'][0]['message'],
        'action': report['findings'][0]['next_action'],
    }


def test_missing_environment_runtime_and_storage_have_stable_codes(tmp_path):
    script_dir = _layout(
        tmp_path,
        source=False,
        runtime_files=('lidarslam_doctor.py', 'missing_helper.py'),
    )
    (script_dir / 'missing_helper.py').unlink()

    report = DOCTOR.build_system_report(
        script_dir=script_dir,
        demo_dir=tmp_path / 'future' / 'demo',
        min_free_space_gib=8,
        environment={},
        command_lookup=lambda _name: None,
        module_available=lambda _name: False,
        disk_usage=_disk(3),
    )

    assert [item['code'] for item in report['findings']] == [
        'product-runtime-incomplete',
        'ros-environment-missing',
        'ros2-cli-missing',
        'rosbag2-python-missing',
        'demo-storage-low',
    ]
    encoded = json.dumps(report, sort_keys=True)
    assert str(tmp_path) not in encoded
    assert report['product']['missing_runtime_files'] == ['missing_helper.py']
    assert report['storage']['additional_bytes_required'] == 5 * DOCTOR.GIB
    assert report['next_action'] == {
        'code': 'product-runtime-incomplete',
        'reason': report['findings'][0]['message'],
        'action': report['findings'][0]['next_action'],
    }
    storage_finding = report['findings'][-1]
    assert '5.00 GiB more' in storage_finding['message']
    assert storage_finding['next_action'].endswith('lidarslam-map doctor')
    assert '<dir>' not in storage_finding['next_action']


def test_storage_shortfall_display_rounds_up_and_human_output_is_actionable(
    tmp_path,
):
    report = _ready_report(tmp_path, source=False)
    report['status'] = 'action_required'
    report['storage']['available_bytes'] = 8 * DOCTOR.GIB - 1
    report['storage']['additional_bytes_required'] = 1
    report['storage']['sufficient_for_fixed_demo'] = False
    report['findings'] = [{
        'code': 'demo-storage-low',
        'message': 'fixture',
        'next_action': 'lidarslam-map doctor',
    }]
    report['next_action'] = {
        'code': 'demo-storage-low',
        'reason': 'fixture',
        'action': 'lidarslam-map doctor',
    }

    rendered = DOCTOR.render_system_report(report)

    assert 'free 0.01 GiB more' in rendered
    assert rendered.count('Do this now:') == 1
    assert 'Do this now:\n  lidarslam-map doctor' in rendered
    assert 'Why: [demo-storage-low] fixture' in rendered


def test_human_action_required_report_prioritizes_one_recovery(tmp_path):
    script_dir = _layout(
        tmp_path,
        source=False,
        runtime_files=('lidarslam_doctor.py', 'missing_helper.py'),
    )
    (script_dir / 'missing_helper.py').unlink()
    report = DOCTOR.build_system_report(
        script_dir=script_dir,
        demo_dir=tmp_path,
        min_free_space_gib=8,
        environment={},
        command_lookup=lambda _name: None,
        module_available=lambda _name: False,
        disk_usage=_disk(3),
    )

    rendered = DOCTOR.render_system_report(report)

    assert rendered.count('Do this now:') == 1
    assert report['next_action']['code'] == 'product-runtime-incomplete'
    assert report['next_action']['action'] in rendered
    assert 'Why: [product-runtime-incomplete]' in rendered
    assert 'Other checks detected:' in rendered
    assert '[ros-environment-missing]' in rendered
    assert '[ros2-cli-missing]' in rendered
    assert '[rosbag2-python-missing]' in rendered
    assert '[demo-storage-low]' in rendered
    assert 'it will choose the next blocker' in rendered
    assert rendered.count('Next:') == 0


def test_schema_binds_next_action_to_report_status(tmp_path):
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    ready = _ready_report(tmp_path, source=False)
    action_required = dict(ready)
    action_required['status'] = 'action_required'
    action_required['findings'] = [{
        'code': 'fixture-action',
        'message': 'fixture reason',
        'next_action': 'lidarslam-map doctor',
    }]

    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(action_required, schema)

    ready['next_action'] = {
        'code': 'fixture-action',
        'reason': 'fixture reason',
        'action': 'lidarslam-map doctor',
    }
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(ready, schema)


def test_unsupported_ros_is_distinct_from_an_unset_environment(tmp_path):
    script_dir = _layout(tmp_path, source=False)

    report = DOCTOR.build_system_report(
        script_dir=script_dir,
        demo_dir=tmp_path,
        min_free_space_gib=1,
        environment={'ROS_DISTRO': 'rolling'},
        command_lookup=lambda _name: '/usr/bin/ros2',
        module_available=lambda _name: True,
        disk_usage=_disk(2),
    )

    assert [item['code'] for item in report['findings']] == [
        'ros-distro-unsupported'
    ]


def test_human_ready_report_leads_to_demo_and_own_bag(tmp_path):
    rendered = DOCTOR.render_system_report(
        _ready_report(tmp_path, source=False)
    )

    assert 'Status:       READY' in rendered
    assert 'Fixed demo: lidarslam-map demo' in rendered
    assert 'Own bag:    lidarslam-map start /path/to/rosbag2' in rendered
    assert 'No launch-file or YAML edits are needed' in rendered
    assert 'stops with a reason code' in rendered
    assert 'used no network and wrote no files' in rendered


def test_bag_mode_delegates_exactly_to_the_existing_preflight(
    tmp_path, monkeypatch
):
    script_dir = tmp_path / 'scripts'
    script_dir.mkdir()
    helper = script_dir / 'preflight_autoware_map_bag.py'
    bag = tmp_path / 'bag with spaces'
    bag.mkdir()
    helper.write_text(
        'import sys\n'
        f'expected_json = [{str(bag)!r}, "--json"]\n'
        f'expected_public = [{str(bag)!r}, "--public-json"]\n'
        'if sys.argv[1:] == expected_json:\n'
        '    raise SystemExit(17)\n'
        'if sys.argv[1:] == expected_public:\n'
        '    raise SystemExit(19)\n'
        'raise SystemExit(18)\n',
        encoding='utf-8',
    )
    monkeypatch.setattr(DOCTOR, 'SCRIPT_DIR', script_dir)

    assert DOCTOR._delegate_bag_doctor(bag, json_output=True) == 17
    assert DOCTOR._delegate_bag_doctor(
        bag,
        json_output=False,
        public_json_output=True,
    ) == 19


def test_help_and_mode_specific_options_fail_closed(tmp_path):
    help_result = subprocess.run(
        [sys.executable, str(SCRIPT), '--help'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0
    assert '[rosbag2_dir]' in help_result.stdout
    assert 'no network' in help_result.stdout
    assert 'writes no files' in help_result.stdout
    assert '--public-json' in help_result.stdout

    mixed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(tmp_path / 'bag'),
            '--demo-dir',
            str(tmp_path),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert mixed.returncode == 2
    assert 'apply only when no rosbag2_dir' in mixed.stderr

    public_without_bag = subprocess.run(
        [sys.executable, str(SCRIPT), '--public-json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert public_without_bag.returncode == 2
    assert '--public-json requires rosbag2_dir' in public_without_bag.stderr


def test_top_level_doctor_without_bag_emits_privacy_bounded_json(tmp_path):
    before = sorted(tmp_path.rglob('*'))
    result = subprocess.run(
        [
            str(CLI),
            'doctor',
            '--json',
            '--demo-dir',
            str(tmp_path),
            '--min-free-space-gib',
            '0.001',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    jsonschema.validate(
        report,
        json.loads(SCHEMA_PATH.read_text(encoding='utf-8')),
    )
    assert report['mode'] == 'system'
    assert report['writes_performed'] is False
    assert str(ROOT) not in result.stdout
    assert str(tmp_path) not in result.stdout
    assert sorted(tmp_path.rglob('*')) == before
