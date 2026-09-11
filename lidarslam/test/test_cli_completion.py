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

"""Tests for the installed Bash completion contract."""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
COMPLETION = (
    REPO_ROOT / 'scripts' / 'completions' / 'lidarslam-map.bash'
)
CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
SHELL_VARIABLE_PATTERN = re.compile(
    r"^(?P<name>_LIDARSLAM_MAP_[A-Z_]+)='(?P<values>[^']*)'$",
    re.MULTILINE,
)


def _completion_variables() -> dict[str, set[str]]:
    """Return the literal command and option inventories in completion."""
    text = COMPLETION.read_text(encoding='utf-8')
    return {
        match.group('name'): set(match.group('values').split())
        for match in SHELL_VARIABLE_PATTERN.finditer(text)
    }


def _complete(*words: str) -> set[str]:
    word_array = ' '.join(f'"{word}"' for word in words)
    script = f"""
source "$1"
COMP_WORDS=({word_array})
COMP_CWORD=$((${{#COMP_WORDS[@]}} - 1))
_lidarslam_map_complete
printf '%s\\n' "${{COMPREPLY[@]}}"
"""
    completed = subprocess.run(
        ['bash', '-c', script, 'bash', str(COMPLETION)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    return set(completed.stdout.splitlines())


def test_completion_is_valid_bash_and_covers_the_option_manifest():
    """Completion inventory should exactly match the public CLI contract."""
    syntax = subprocess.run(
        ['bash', '-n', str(COMPLETION)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))
    variables = _completion_variables()
    assert variables['_LIDARSLAM_MAP_COMMANDS'] == set(contract['commands'])
    assert variables['_LIDARSLAM_MAP_GLOBAL_OPTIONS'] == {
        name
        for option in contract['global_options']
        for name in option['names']
        if name.startswith('--')
    }
    for command, command_contract in contract['commands'].items():
        variable = (
            '_LIDARSLAM_MAP_'
            + command.upper().replace('-', '_')
            + '_OPTIONS'
        )
        expected = {
            name
            for option in command_contract['options']
            for name in option['names']
            if name.startswith('--')
        }
        assert variables[variable] == expected


def test_completion_suggests_commands_options_and_bounded_choices():
    """Completion should understand command context and finite option values."""
    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))

    assert {
        'doctor',
        'run',
        'inspect',
        'view',
        'migrate-manifest',
        'rollback-plan',
    } <= _complete(
        'lidarslam-map',
        '',
    )
    assert _complete('lidarslam-map', 'run', '--ver') == {'--verification'}
    for command, value_options in contract['value_options'].items():
        for option, value_contract in value_options.items():
            if value_contract['kind'] == 'enum':
                assert _complete(
                    'lidarslam-map',
                    command,
                    option,
                    '',
                ) == set(value_contract['choices'])
