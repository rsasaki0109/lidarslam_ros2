#!/usr/bin/env python3
"""Resolve a locally available credential for bounded GitHub API reads."""

from __future__ import annotations

import os
import subprocess
from functools import lru_cache
from urllib.parse import urlsplit


GITHUB_API_HOST = 'api.github.com'
GH_AUTH_TIMEOUT_SECONDS = 5


def is_github_api_url(url: str) -> bool:
    """Return whether *url* is the exact public GitHub HTTPS API origin."""
    try:
        parsed = urlsplit(url)
        port = parsed.port
    except (TypeError, ValueError):
        return False
    return (
        parsed.scheme == 'https'
        and parsed.hostname == GITHUB_API_HOST
        and port in (None, 443)
        and parsed.username is None
        and parsed.password is None
    )


def _valid_token(value: object) -> str | None:
    """Reject empty or header-unsafe credential text."""
    if not isinstance(value, str) or not value:
        return None
    if value != value.strip() or '\r' in value or '\n' in value:
        return None
    return value


@lru_cache(maxsize=1)
def _stored_gh_token() -> str | None:
    """Read the active gh credential once without prompting or printing it."""
    try:
        result = subprocess.run(
            ['gh', 'auth', 'token'],
            check=False,
            capture_output=True,
            text=True,
            stdin=subprocess.DEVNULL,
            timeout=GH_AUTH_TIMEOUT_SECONDS,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return _valid_token(result.stdout.strip())


def github_api_authorization(
    url: str,
    *,
    method: str,
) -> dict[str, str]:
    """Return a bearer header only for an exact public GitHub API GET."""
    if method != 'GET' or not is_github_api_url(url):
        return {}
    token = _valid_token(os.environ.get('GITHUB_TOKEN'))
    if token is None:
        token = _stored_gh_token()
    if token is None:
        return {}
    return {'Authorization': f'Bearer {token}'}
