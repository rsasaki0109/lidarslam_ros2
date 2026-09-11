#!/usr/bin/env python3
"""Download a file until its exact size and checksum match expected values.

This is intentionally restart-only.  Some Dataverse endpoints advertise Range
support but answer range requests with ``200 OK`` and the complete object from
byte zero.  Appending such a response would silently corrupt the archive.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def digest(path: Path, algorithm: str) -> str:
    checksum = hashlib.new(algorithm)
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            checksum.update(block)
    return checksum.hexdigest()


def matches(path: Path, expected_size: int, algorithm: str,
            expected_digest: str) -> tuple[bool, dict[str, object]]:
    record: dict[str, object] = {
        'path': str(path),
        'exists': path.is_file(),
    }
    if not path.is_file():
        return False, record
    size = path.stat().st_size
    record['size_bytes'] = size
    if size != expected_size:
        record['reason'] = 'size_mismatch'
        return False, record
    actual_digest = digest(path, algorithm)
    record[f'{algorithm}'] = actual_digest
    if actual_digest.lower() != expected_digest.lower():
        record['reason'] = 'checksum_mismatch'
        return False, record
    record['reason'] = 'verified'
    return True, record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-size', type=int, required=True)
    parser.add_argument('--checksum-algorithm', choices=('md5', 'sha256'),
                        default='md5')
    parser.add_argument('--expected-checksum', required=True)
    parser.add_argument('--retry-delay-seconds', type=float, default=5.0)
    parser.add_argument('--max-attempts', type=int, default=0,
                        help='0 retries forever')
    args = parser.parse_args()

    if args.expected_size <= 0:
        parser.error('--expected-size must be positive')
    if args.retry_delay_seconds < 0:
        parser.error('--retry-delay-seconds must be non-negative')

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    part = output.with_name(output.name + '.part')

    valid, record = matches(
        output, args.expected_size, args.checksum_algorithm,
        args.expected_checksum)
    if valid:
        print(json.dumps({'status': 'already_verified', **record}, sort_keys=True))
        return 0

    attempt = 0
    while args.max_attempts == 0 or attempt < args.max_attempts:
        attempt += 1
        part.unlink(missing_ok=True)
        print(json.dumps({
            'status': 'downloading', 'attempt': attempt,
            'url': args.url, 'part': str(part)}, sort_keys=True), flush=True)
        result = subprocess.run([
            'curl', '--http1.1', '--fail', '--location', '--silent',
            '--show-error',
            '--retry', '5', '--retry-all-errors',
            '--user-agent', 'Mozilla/5.0',
            '--output', str(part), args.url])
        valid, record = matches(
            part, args.expected_size, args.checksum_algorithm,
            args.expected_checksum)
        print(json.dumps({
            'status': 'attempt_complete', 'attempt': attempt,
            'curl_returncode': result.returncode, **record}, sort_keys=True),
            flush=True)
        if valid:
            os.replace(part, output)
            print(json.dumps({
                'status': 'verified', 'attempt': attempt,
                'path': str(output),
                'size_bytes': args.expected_size,
                args.checksum_algorithm: args.expected_checksum.lower(),
            }, sort_keys=True), flush=True)
            return 0
        if args.max_attempts == 0 or attempt < args.max_attempts:
            time.sleep(args.retry_delay_seconds)

    print('error: no verified download after maximum attempts', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
