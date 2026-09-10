#!/usr/bin/env python3
"""Regression tests for exact Docker missing-container diagnostics."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO = Path(__file__).resolve().parents[2]
CAMPAIGN = REPO / 'scripts' / 'run_registration_plugin_release_campaign.py'
LEG = REPO / 'scripts' / 'run_registration_plugin_release_leg.py'


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def campaign_module():
    return _load(CAMPAIGN, 'registration_plugin_release_campaign_absence_test')


@pytest.fixture(scope='module')
def leg_module():
    return _load(LEG, 'registration_plugin_release_leg_absence_test')


@pytest.mark.parametrize(
    'message',
    [
        'Error: No such object: fixture-container',
        'Error: No such container: fixture-container',
        'error: no such object: fixture-container',
        'error: no such container: fixture-container',
    ],
)
@pytest.mark.parametrize('newline', [b'', b'\n'])
@pytest.mark.parametrize('stdout', [b'', b'[]\n'])
def test_campaign_accepts_only_exact_missing_container_message(
        campaign_module, monkeypatch, message, newline, stdout):
    def inspect(_argv, **_kwargs):
        return SimpleNamespace(
            returncode=1, stdout=stdout, stderr=message.encode('ascii') + newline
        )

    monkeypatch.setattr(campaign_module.subprocess, 'run', inspect)
    assert campaign_module._container_probe_default('fixture-container') is None


@pytest.mark.parametrize(
    'stdout,stderr,returncode',
    [
        (b'', b'error: no such object: other-container\n', 1),
        (b'[]', b'error: no such object: fixture-container\n', 1),
        (b'[]\n\n', b'error: no such object: fixture-container\n', 1),
        (b'[]\n[]\n', b'error: no such object: fixture-container\n', 1),
        (b'extra\n', b'error: no such object: fixture-container\n', 1),
        (b'', b'error: no such object: fixture-container\nwarning\n', 1),
        (b'', b'daemon unavailable\n', 1),
        (b'', b'error: no such object: fixture-container\n', 2),
    ],
)
def test_campaign_rejects_wrong_or_ambiguous_absence(
        campaign_module, monkeypatch, stdout, stderr, returncode):
    def inspect(_argv, **_kwargs):
        return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)

    monkeypatch.setattr(campaign_module.subprocess, 'run', inspect)
    with pytest.raises(campaign_module.CampaignError):
        campaign_module._container_probe_default('fixture-container')


@pytest.mark.parametrize(
    'message',
    [
        b'Error: No such object: fixture-container',
        b'error: no such object: fixture-container',
        b'[]\nError: No such object: fixture-container',
        b'[]\nerror: no such object: fixture-container',
    ],
)
def test_leg_accepts_exact_lowercase_or_legacy_absence(leg_module, message):
    assert leg_module._validate_absence_output(message, 'fixture-container') is None
    assert leg_module._validate_absence_output(message + b'\n', 'fixture-container') is None


@pytest.mark.parametrize(
    'message',
    [
        b'error: no such object: other-container\n',
        b'[]error: no such object: fixture-container\n',
        b'[]\n\nerror: no such object: fixture-container\n',
        b'[]\n[]\nerror: no such object: fixture-container\n',
        b'error: no such object: fixture-container\nextra\n',
        b'warning\nerror: no such object: fixture-container\n',
        b'error: no such object: fixture-container\xff\n',
    ],
)
def test_leg_rejects_wrong_or_ambiguous_absence(leg_module, message):
    with pytest.raises(leg_module.LauncherError):
        leg_module._validate_absence_output(message, 'fixture-container')
