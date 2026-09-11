#!/usr/bin/env python3
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

"""Prepare a fail-closed template for one usability scorecard trial.

The generated JSON is a recording worksheet, not evidence of a completed
trial. Measurements, commands, transcripts, and task checks stay empty or
negative until an observer records the real run. The template is deliberately
safe to pass to ``check_usability_scorecard.py``: its incomplete state remains
visible instead of being treated as a successful comparison.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from check_usability_scorecard import (  # noqa: E402
    ScorecardError,
    TASK_CONTRACTS,
    TRIAL_SCHEMA_URI,
    validate_trial,
)


SHA256 = re.compile(r'^[0-9a-f]{64}$')


def _now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _slug(value: str, label: str) -> str:
    if not re.fullmatch(r'[a-z0-9][a-z0-9._-]{2,79}', value):
        raise ScorecardError(
            f'{label} must be a lowercase slug (3-80 characters): {value!r}'
        )
    return value


def build_template(args: argparse.Namespace) -> dict:
    """Build a schema-valid, explicitly incomplete trial worksheet."""
    trial_id = args.trial_id or (
        f'ux-template-{args.product}-{datetime.now(timezone.utc):%Y%m%d}'
    )
    _slug(trial_id, 'trial-id')
    _slug(args.cohort_id, 'cohort-id')
    _slug(args.comparison_pair_id, 'comparison-pair-id')
    _slug(args.input_id, 'input-id')
    _slug(args.hardware_class, 'hardware-class')
    if not SHA256.fullmatch(args.machine_fingerprint_sha256):
        raise ScorecardError(
            'machine-fingerprint-sha256 must be 64 lowercase hexadecimal '
            'characters'
        )

    tasks = []
    for contract in TASK_CONTRACTS:
        tasks.append({
            'task_id': contract['task_id'],
            'documentation_url': args.documentation_url,
            'input_id': args.input_id,
            'exact_commands': [],
            'measurements': {
                'wall_time_sec': None,
                'active_operator_time_sec': None,
                'command_count': None,
                'workflow_download_bytes': None,
                'peak_disk_bytes': None,
                'failure_count': None,
                'output_bytes': None,
            },
            'checks': [
                {'id': check_id, 'passed': False}
                for check_id in contract['checks']
            ],
            'outcome': {
                'status': 'FAIL',
                'undocumented_manual_steps': 0,
                'finding_codes': ['not-recorded'],
            },
            'evidence': {
                'transcript_sha256': None,
                'public_url': None,
            },
        })

    record = {
        'schema_version': 1,
        'schema_uri': TRIAL_SCHEMA_URI,
        'trial_id': trial_id,
        'captured_at': args.captured_at or _now(),
        'product': {
            'id': args.product,
            'version': args.version,
            'revision': {
                'kind': args.revision_kind,
                'value': args.revision,
            },
            'documentation_root_url': args.documentation_url,
            'publicly_resolvable': args.publicly_resolvable,
        },
        'operator': {
            'class': args.operator_class,
            'cohort_id': args.cohort_id,
            'first_attempt': not args.not_first_attempt,
            'product_order': args.product_order,
        },
        'environment': {
            'comparison_pair_id': args.comparison_pair_id,
            'clean_start': args.clean_start,
            'supported_by_product_docs': True,
            'ros_distro': args.ros_distro,
            'os_family': args.os_family,
            'architecture': args.architecture,
            'hardware_class': args.hardware_class,
            'machine_fingerprint_sha256': args.machine_fingerprint_sha256,
        },
        'tasks': tasks,
        'privacy': {
            'contains_private_paths': False,
            'contains_operator_identity': False,
            'contains_secrets': False,
            'review_before_sharing': True,
        },
    }
    validate_trial(record)
    return record


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--product', choices=('lidarslam_ros2', 'glim'),
                        required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument(
        '--revision-kind',
        choices=('git-commit', 'release-tag', 'image-digest'),
        required=True,
    )
    parser.add_argument('--revision', required=True)
    parser.add_argument('--documentation-url', required=True)
    parser.add_argument('--trial-id')
    parser.add_argument('--captured-at')
    parser.add_argument('--cohort-id', required=True)
    parser.add_argument('--comparison-pair-id', required=True)
    parser.add_argument('--input-id', required=True)
    parser.add_argument('--product-order', choices=('first', 'second'),
                        required=True)
    parser.add_argument('--operator-class', choices=('maintainer', 'external'),
                        default='external')
    parser.add_argument('--not-first-attempt', action='store_true')
    parser.add_argument('--publicly-resolvable', action='store_true')
    parser.add_argument('--clean-start', action='store_true')
    parser.add_argument('--ros-distro', choices=('humble', 'jazzy'),
                        required=True)
    parser.add_argument(
        '--os-family',
        choices=('ubuntu-22.04', 'ubuntu-24.04'),
        required=True,
    )
    parser.add_argument('--architecture', choices=('x86_64', 'aarch64'),
                        required=True)
    parser.add_argument('--hardware-class', required=True)
    parser.add_argument('--machine-fingerprint-sha256', required=True)
    parser.add_argument(
        '--output',
        type=Path,
        help='Write once to this path; otherwise emit JSON on stdout.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Write one incomplete worksheet without overwriting an existing file."""
    args = _parser().parse_args(argv)
    try:
        record = build_template(args)
        payload = json.dumps(record, indent=2, sort_keys=True) + '\n'
        if args.output is None:
            sys.stdout.write(payload)
        else:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(payload)
            print(
                f'Wrote incomplete scorecard template: {args.output}',
                file=sys.stderr,
            )
            print(
                'Do not add it to the reviewed evidence index until the '
                'observed trial is complete.',
                file=sys.stderr,
            )
    except (OSError, ScorecardError) as exc:
        print(f'usability scorecard template error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
