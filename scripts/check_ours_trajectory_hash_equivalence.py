#!/usr/bin/env python3
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
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
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

"""Compare two opaque trajectory artifacts by streaming SHA-256 only."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def compare(baseline: Path, instrumented: Path) -> dict[str, object]:
    if not baseline.is_file() or not instrumented.is_file():
        raise FileNotFoundError('both trajectory artifacts must be regular files')
    baseline_sha = sha256_file(baseline)
    instrumented_sha = sha256_file(instrumented)
    return {
        'schema_version': 1,
        'kind': 'ours_trajectory_hash_equivalence',
        'status': 'pass' if baseline_sha == instrumented_sha else 'invalid',
        'payload_opened_for_metrics': False,
        'byte_identical': baseline_sha == instrumented_sha,
        'baseline_path': str(baseline),
        'instrumented_path': str(instrumented),
        'baseline_sha256': baseline_sha,
        'instrumented_sha256': instrumented_sha,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('instrumented', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = compare(args.baseline, args.instrumented)
    except (OSError, ValueError) as error:
        result = {
            'schema_version': 1,
            'kind': 'ours_trajectory_hash_equivalence',
            'status': 'invalid',
            'payload_opened_for_metrics': False,
            'byte_identical': False,
            'error': str(error),
        }
    encoded = json.dumps(result, sort_keys=True, separators=(',', ':')) + '\n'
    if args.output is None:
        print(encoded, end='')
    else:
        args.output.write_text(encoded, encoding='utf-8')
    return 0 if result['status'] == 'pass' else 2


if __name__ == '__main__':
    raise SystemExit(main())
