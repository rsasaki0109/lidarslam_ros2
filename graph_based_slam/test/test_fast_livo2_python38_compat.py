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

"""Python 3.8 compatibility checks for the container feeder path helpers."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FEEDER = ROOT / 'scripts/fast_livo2_m6a10_feeder.py'


def _load_relative_helper():
    tree = ast.parse(FEEDER.read_text(encoding='utf-8'))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == '_is_relative_to')
    namespace = {'Path': Path}
    module = ast.Module(body=[function], type_ignores=[])
    exec(compile(module, str(FEEDER), 'exec'), namespace)  # noqa: S102
    return namespace['_is_relative_to']


def test_relative_helper_supports_python38_api():
    helper = _load_relative_helper()
    root = Path('/tmp/output')
    assert helper(root / 'feeder_progress.json', root)
    assert helper(root, root)
    assert not helper(Path('/tmp/output-other/file'), root)


def test_feeder_does_not_use_python39_only_path_method():
    source = FEEDER.read_text(encoding='utf-8')
    assert '.is_relative_to(' not in source
    assert 'path.relative_to(parent)' in source
