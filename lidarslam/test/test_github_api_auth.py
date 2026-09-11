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

"""Tests for scoped, non-interactive GitHub API credential discovery."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'github_api_auth.py'
SPEC = importlib.util.spec_from_file_location('github_api_auth', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUTH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUTH)


def _clear_cache() -> None:
    AUTH._stored_gh_token.cache_clear()


def test_explicit_token_is_used_only_for_exact_github_api_origin(monkeypatch):
    """An explicit token cannot escape the exact GitHub API origin."""
    monkeypatch.setenv('GITHUB_TOKEN', 'explicit-read-token')
    monkeypatch.setattr(
        AUTH.subprocess,
        'run',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('gh credential discovery was unexpectedly used')
        ),
    )
    _clear_cache()

    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/repo',
        method='GET',
    ) == {'Authorization': 'Bearer explicit-read-token'}
    assert AUTH.github_api_authorization(
        'https://raw.githubusercontent.com/owner/repo/main/file',
        method='GET',
    ) == {}
    assert AUTH.github_api_authorization(
        'https://api.github.com.evil.example/repos/owner/repo',
        method='GET',
    ) == {}
    assert AUTH.github_api_authorization(
        'https://api.github.com@evil.example/repos/owner/repo',
        method='GET',
    ) == {}
    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/repo',
        method='POST',
    ) == {}


def test_exact_origin_predicate_rejects_lookalikes_and_non_https():
    """The reusable URL predicate accepts only the public HTTPS API."""
    assert AUTH.is_github_api_url(
        'https://api.github.com/repos/owner/repo'
    ) is True
    assert AUTH.is_github_api_url(
        'https://api.github.com:443/repos/owner/repo'
    ) is True
    assert AUTH.is_github_api_url(
        'http://api.github.com/repos/owner/repo'
    ) is False
    assert AUTH.is_github_api_url(
        'https://api.github.com:444/repos/owner/repo'
    ) is False
    assert AUTH.is_github_api_url(
        'https://api.github.com.evil.example/repos/owner/repo'
    ) is False


def test_stored_gh_token_is_discovered_once_without_prompting(monkeypatch):
    """The active gh credential is reused without an interactive prompt."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(
            command,
            0,
            stdout='stored-read-token\n',
            stderr='',
        )

    monkeypatch.delenv('GITHUB_TOKEN', raising=False)
    monkeypatch.setattr(AUTH.subprocess, 'run', fake_run)
    _clear_cache()

    expected = {'Authorization': 'Bearer stored-read-token'}
    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/repo',
        method='GET',
    ) == expected
    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/other',
        method='GET',
    ) == expected
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command == ['gh', 'auth', 'token']
    assert kwargs['check'] is False
    assert kwargs['capture_output'] is True
    assert kwargs['text'] is True
    assert kwargs['stdin'] is subprocess.DEVNULL
    assert kwargs['timeout'] == 5


def test_missing_or_header_unsafe_credential_falls_back_to_no_header(
    monkeypatch,
):
    """Header-unsafe credential output is never attached to a request."""
    monkeypatch.delenv('GITHUB_TOKEN', raising=False)
    monkeypatch.setattr(
        AUTH.subprocess,
        'run',
        lambda command, **_kwargs: subprocess.CompletedProcess(
            command,
            0,
            stdout='unsafe\ncredential\n',
            stderr='',
        ),
    )
    _clear_cache()

    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/repo',
        method='GET',
    ) == {}


def test_failed_gh_lookup_is_a_bounded_anonymous_fallback(monkeypatch):
    """Missing local authentication preserves the anonymous read path."""
    monkeypatch.delenv('GITHUB_TOKEN', raising=False)
    monkeypatch.setattr(
        AUTH.subprocess,
        'run',
        lambda command, **_kwargs: subprocess.CompletedProcess(
            command,
            1,
            stdout='',
            stderr='not logged in',
        ),
    )
    _clear_cache()

    assert AUTH.github_api_authorization(
        'https://api.github.com/repos/owner/repo',
        method='GET',
    ) == {}
