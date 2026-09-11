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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Regression tests for the recommended RKO-LIO launch defaults."""

from __future__ import annotations

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PATH = REPO_ROOT / 'lidarslam' / 'launch' / 'rko_lio_slam.launch.py'


def _parse_launch_ast() -> ast.Module:
    return ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'))


def _constant_string(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _find_function(module: ast.Module, name: str) -> ast.FunctionDef:
    for node in module.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f'function not found: {name}')


def _find_declare_launch_argument_call(
    launch_description: ast.Call,
    argument_name: str,
) -> ast.Call:
    for item in launch_description.args[0].elts:
        if not isinstance(item, ast.Call):
            continue
        if not isinstance(item.func, ast.Name):
            continue
        if item.func.id != 'DeclareLaunchArgument':
            continue
        if not item.args:
            continue
        if _constant_string(item.args[0]) == argument_name:
            return item
    raise AssertionError(f'DeclareLaunchArgument not found: {argument_name}')


def test_base_frame_defaults_to_base_link():
    """The default base frame should remain base_link for NTU VIRAL."""
    module = _parse_launch_ast()
    generate_launch_description = _find_function(
        module,
        'generate_launch_description',
    )
    return_stmt = next(
        node for node in generate_launch_description.body
        if isinstance(node, ast.Return)
    )
    assert isinstance(return_stmt.value, ast.Call)
    base_frame_call = _find_declare_launch_argument_call(
        return_stmt.value,
        'base_frame',
    )
    default_kw = next(
        kw for kw in base_frame_call.keywords if kw.arg == 'default_value'
    )
    assert _constant_string(default_kw.value) == 'base_link'


def test_rko_param_file_is_applied_after_launch_defaults():
    """The optional YAML must be appended after inline defaults to override them."""
    module = _parse_launch_ast()
    create_node = _find_function(module, 'create_rko_offline_node')

    append_values: list[str] = []

    class AppendCollector(ast.NodeVisitor):
        def visit_Call(self, node: ast.Call) -> None:
            if (
                isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'parameters'
                and node.func.attr == 'append'
                and len(node.args) == 1
            ):
                arg = node.args[0]
                if isinstance(arg, ast.Dict):
                    append_values.append('dict')
                elif isinstance(arg, ast.Name):
                    append_values.append(arg.id)
            self.generic_visit(node)

    AppendCollector().visit(create_node)

    assert append_values[:2] == ['dict', 'rko_param_file']


def test_graph_param_overrides_are_optional():
    """Graph launch arguments must not override custom YAML by default."""
    module = _parse_launch_ast()
    generate_launch_description = _find_function(
        module,
        'generate_launch_description',
    )
    return_stmt = next(
        node for node in generate_launch_description.body
        if isinstance(node, ast.Return)
    )
    assert isinstance(return_stmt.value, ast.Call)

    optional_overrides = [
        'adjacent_edge_info_weight',
        'use_scan_context',
        'use_pcd_cache',
        'threshold_loop_closure_score',
        'distance_loop_closure',
    ]
    for argument_name in optional_overrides:
        arg_call = _find_declare_launch_argument_call(
            return_stmt.value,
            argument_name,
        )
        default_kw = next(
            kw for kw in arg_call.keywords if kw.arg == 'default_value'
        )
        assert _constant_string(default_kw.value) == ''


def test_rko_graph_backend_receives_gnss_topic_override():
    """RKO-LIO launch should pass gnss_topic through to graph_based_slam."""
    module = _parse_launch_ast()
    create_graph_node = _find_function(module, 'create_graph_based_slam_node')

    parameters_assign = next(
        node for node in create_graph_node.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'parameters'
            for target in node.targets
        )
    )
    assert isinstance(parameters_assign.value, ast.List)
    dict_nodes = [elt for elt in parameters_assign.value.elts if isinstance(elt, ast.Dict)]
    assert dict_nodes
    launch_dict = dict_nodes[0]

    found = False
    for key, value in zip(launch_dict.keys, launch_dict.values):
        if not isinstance(key, ast.Constant) or key.value != 'gnss_topic':
            continue
        if not isinstance(value, ast.Call):
            continue
        if not isinstance(value.func, ast.Name) or value.func.id != 'LaunchConfiguration':
            continue
        if not value.args or not isinstance(value.args[0], ast.Constant):
            continue
        if value.args[0].value == 'gnss_topic':
            found = True
            break

    assert found


def test_rko_launch_declares_gnss_topic_argument():
    """RKO-LIO launch should expose a gnss_topic argument."""
    module = _parse_launch_ast()
    generate_launch_description = _find_function(
        module,
        'generate_launch_description',
    )
    return_stmt = next(
        node for node in generate_launch_description.body
        if isinstance(node, ast.Return)
    )
    assert isinstance(return_stmt.value, ast.Call)
    gnss_topic_call = _find_declare_launch_argument_call(
        return_stmt.value,
        'gnss_topic',
    )
    default_kw = next(
        kw for kw in gnss_topic_call.keywords if kw.arg == 'default_value'
    )
    assert _constant_string(default_kw.value) == '/gnss/fix'


def test_offline_node_pins_odom_topic():
    """The frontend must publish odometry on the fork's historical topic."""
    # The upstream-integrated rko_lio renamed its default odometry topic from
    # "rko_lio/odometry" to "rko_lio/odom".  The graph backend's odom_input
    # remap (and the record_backend_input tooling) still expect
    # "/rko_lio/odometry", so the launch must pin odom_topic explicitly;
    # otherwise the backend silently receives no odometry and saves an empty
    # map.
    module = _parse_launch_ast()
    factory = _find_function(module, 'create_rko_offline_node')

    found = False
    for launch_dict in (n for n in ast.walk(factory) if isinstance(n, ast.Dict)):
        for key, value in zip(launch_dict.keys, launch_dict.values):
            if _constant_string(key) != 'odom_topic':
                continue
            assert _constant_string(value) == '/rko_lio/odometry'
            found = True
    assert found


def test_graph_backend_odom_remap_matches_frontend_topic():
    """The backend odom_input remap must target the pinned frontend topic."""
    module = _parse_launch_ast()
    factory = _find_function(module, 'create_graph_based_slam_node')

    remap_pairs = []
    for call in (n for n in ast.walk(factory) if isinstance(n, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        remappings_kw = next(
            (kw for kw in call.keywords if kw.arg == 'remappings'), None
        )
        if remappings_kw is None:
            continue
        remap_pairs.extend(
            tuple(_constant_string(elt) for elt in pair.elts)
            for pair in remappings_kw.value.elts
            if isinstance(pair, ast.Tuple)
        )
    assert ('odom_input', '/rko_lio/odometry') in remap_pairs


def test_offline_start_barrier_is_opt_in_and_fail_closed():
    module = _parse_launch_ast()
    generate_launch_description = _find_function(
        module,
        'generate_launch_description',
    )
    return_stmt = next(
        node for node in generate_launch_description.body
        if isinstance(node, ast.Return)
    )
    assert isinstance(return_stmt.value, ast.Call)

    expected_defaults = {
        'wait_for_output_subscribers': 'false',
        'min_odom_subscribers': '1',
        'min_deskewed_scan_subscribers': '1',
        'subscriber_wait_timeout_sec': '30.0',
        'subscriber_settle_polls': '3',
    }
    for argument_name, expected_default in expected_defaults.items():
        argument = _find_declare_launch_argument_call(
            return_stmt.value,
            argument_name,
        )
        default_kw = next(
            kw for kw in argument.keywords if kw.arg == 'default_value'
        )
        assert _constant_string(default_kw.value) == expected_default

    launch_source = LAUNCH_PATH.read_text(encoding='utf-8')
    assert 'wait_for_offline_output_subscribers.sh' in launch_source
    assert "'/rko_lio/odometry'" in launch_source
    assert "'/rko_lio/frame'" in launch_source
    assert 'prefix = shlex.join([' in launch_source
    assert "'offline.wait_for_output_subscribers': True" in launch_source
    assert "'offline.min_odom_subscribers': min_odom_subscribers" in launch_source
    assert "'offline.min_deskewed_subscribers': min_deskewed_subscribers" in launch_source
    assert "'offline.subscriber_wait_timeout_ms': subscriber_wait_timeout_ms" in launch_source
    assert "'offline.subscriber_settle_polls': subscriber_settle_polls" in launch_source
    assert 'prefix=prefix' in launch_source
