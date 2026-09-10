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

"""Small GNU-time-compatible fallback for images without ``/usr/bin/time``."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    return parser


def main() -> int:
    args = _parser().parse_args()
    command = list(args.command)
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        raise SystemExit('a command is required after --')
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started = time.monotonic()
    process = subprocess.Popen(command, start_new_session=True)

    def write_report(return_code: int) -> None:
        elapsed = time.monotonic() - started
        after = resource.getrusage(resource.RUSAGE_CHILDREN)
        report = (
            f'User time (seconds): {after.ru_utime - before.ru_utime:.6f}\n'
            f'System time (seconds): {after.ru_stime - before.ru_stime:.6f}\n'
            f'Elapsed (wall clock) time (seconds): {elapsed:.6f}\n'
            f'Maximum resident set size (kbytes): {after.ru_maxrss}\n'
            f'File system inputs: {after.ru_inblock - before.ru_inblock}\n'
            f'File system outputs: {after.ru_oublock - before.ru_oublock}\n'
            f'Exit status: {return_code if return_code >= 0 else 128 - return_code}\n'
        )
        part = args.output.with_name(args.output.name + '.part')
        part.write_text(report, encoding='utf-8')
        part.replace(args.output)

    interrupted_signal: list[int] = []

    def on_signal(signum: int, _frame: object) -> None:
        interrupted_signal.append(signum)
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass
        # ``Popen.wait`` is already active in the main thread; re-entering it
        # from this handler can deadlock on Popen's waitpid lock.  Force the
        # isolated workload group down and let the outer wait reap it, after
        # which the normal report path below remains atomic.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    return_code = process.wait()
    write_report(
        128 + interrupted_signal[0]
        if interrupted_signal
        else return_code
    )
    if interrupted_signal:
        return 128 + interrupted_signal[0]
    if return_code < 0:
        return 128 - return_code
    return return_code


if __name__ == '__main__':
    sys.exit(main())
