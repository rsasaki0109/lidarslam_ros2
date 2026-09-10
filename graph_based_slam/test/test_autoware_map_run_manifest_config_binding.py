"""Effective-config binding tests for the public map runner."""

from __future__ import annotations

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
MANIFEST_PATH = REPO_ROOT / 'scripts' / 'map_run_manifest.py'


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_rko_plan_makes_both_effective_configs_explicit(
    tmp_path: Path, monkeypatch,
):
    runner = _load('run_autoware_map_from_bag_config_test', RUNNER_PATH)
    manifest = _load('map_run_manifest_config_test', MANIFEST_PATH)
    payload = {
        'recommended_profile_id': 'rko_lio_graph_public_path',
        'recommendations': [{
            'id': 'rko_lio_graph_public_path',
            'label': 'RKO-LIO graph public path',
        }],
        'summary': {
            'bag_path': '/fixtures/ntu_viral',
            'topics': {
                'pointcloud2': [{'name': '/points'}],
                'imu': [{'name': '/imu'}],
            },
        },
    }

    class Preflight:
        @staticmethod
        def build_preflight_payload(_bag_path: Path):
            return payload

    monkeypatch.setattr(runner, '_load_script_module', lambda *_args: Preflight)
    bag = tmp_path / 'bag'
    bag.mkdir()
    plan = runner.build_execution_plan(
        bag_path=bag,
        profile_id=None,
        output_dir=tmp_path / 'output',
        verify_map=True,
    )

    command = plan['command']
    lidarslam_config = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'
    rko_config = REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_ntu_viral.yaml'
    assert command[command.index('--lidarslam-param') + 1] == str(lidarslam_config)
    assert command[command.index('--rko-param') + 1] == str(rko_config)
    identities = manifest._config_identities(command, REPO_ROOT)
    assert [item['path'] for item in identities] == [
        'lidarslam/param/lidarslam.yaml',
        'lidarslam/param/rko_lio_ntu_viral.yaml',
    ]
    assert all(len(item['sha256']) == 64 for item in identities)
