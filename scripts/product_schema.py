#!/usr/bin/env python3
"""Locate and validate installed or source-tree product JSON contracts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from jsonschema import Draft7Validator, FormatChecker


SCRIPT_DIR = Path(__file__).resolve().parent


def schema_path(filename: str) -> Path:
    """Return a product schema from a source checkout or installed package."""
    candidates = (
        SCRIPT_DIR.parent / 'docs' / 'schemas' / filename,
        SCRIPT_DIR.parent / 'schemas' / filename,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    rendered = ', '.join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f'product schema {filename!r} was not found; checked: {rendered}'
    )


def load_json_object(path: Path, label: str) -> dict[str, Any]:
    """Read a JSON object with a concise product-facing error."""
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'{label} is not readable JSON: {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError(f'{label} root must be a JSON object: {path}')
    return payload


def validate_contract(payload: dict[str, Any], filename: str) -> None:
    """Validate a payload against an installed Draft 7 schema."""
    schema = load_json_object(schema_path(filename), 'product schema')
    Draft7Validator.check_schema(schema)
    errors = sorted(
        Draft7Validator(
            schema,
            format_checker=FormatChecker(),
        ).iter_errors(payload),
        key=lambda error: tuple(str(item) for item in error.absolute_path),
    )
    if errors:
        error = errors[0]
        location = '/'.join(str(item) for item in error.absolute_path)
        prefix = f' at {location}' if location else ''
        raise ValueError(
            f'{filename} validation failed{prefix}: {error.message}'
        )
