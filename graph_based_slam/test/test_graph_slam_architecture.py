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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Executable dependency and parity contracts for the Graph SLAM boundary."""

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
INCLUDE_ROOT = PACKAGE_ROOT / 'include'
SOURCE_ROOT = PACKAGE_ROOT / 'src'
APPLICATION_HEADER = (
    INCLUDE_ROOT / 'graph_based_slam' / 'graph_slam_application.hpp'
)
APPLICATION_SOURCE = SOURCE_ROOT / 'graph_slam_application.cpp'
COMPONENT_SOURCE = SOURCE_ROOT / 'graph_based_slam_component.cpp'
LIVE_ADAPTER_SOURCE = SOURCE_ROOT / 'graph_slam_ros_adapter.cpp'
OFFLINE_ADAPTER_SOURCE = SOURCE_ROOT / 'graph_slam_offline_runner.cpp'
CMAKE = PACKAGE_ROOT / 'CMakeLists.txt'


def _local_include(source: Path, include: str) -> Path | None:
    candidates = [source.parent / include, INCLUDE_ROOT / include,
                  SOURCE_ROOT / include]
    for candidate in candidates:
        if candidate.is_file() and PACKAGE_ROOT in candidate.resolve().parents:
            return candidate.resolve()
    return None


def _application_source_closure() -> set[Path]:
    pending = [APPLICATION_HEADER.resolve(), APPLICATION_SOURCE.resolve()]
    visited: set[Path] = set()
    include_pattern = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.MULTILINE)
    while pending:
        source = pending.pop()
        if source in visited:
            continue
        visited.add(source)
        text = source.read_text(encoding='utf-8')
        for include in include_pattern.findall(text):
            dependency = _local_include(source, include)
            if dependency is not None and dependency not in visited:
                pending.append(dependency)
    return visited


def _without_comments(text: str) -> str:
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return re.sub(r'//.*', '', text)


def test_ros_component_is_a_small_registration_shell():
    source = COMPONENT_SOURCE.read_text(encoding='utf-8')
    assert len(source.splitlines()) <= 300
    assert 'RCLCPP_COMPONENTS_REGISTER_NODE' in source
    assert 'GraphBasedSlamComponent::GraphBasedSlamComponent' in source
    assert 'GraphSlamApplication' not in source
    assert 'std::filesystem' not in source


def test_application_dependency_closure_has_no_external_io_or_ros():
    forbidden_includes = re.compile(
        r'#\s*include\s*[<"](?:rclcpp|rcl|rmw|rosbag2|tf2_ros|message_filters|'
        r'filesystem|fstream)(?:[/>"])'
    )
    forbidden_symbols = re.compile(
        r'\brclcpp::|\brcl_|\brmw_|\brosbag2_|\bstd::filesystem|'
        r'\bstd::(?:i|o|f)fstream|'
        r'\bstd::chrono::(?:system_clock|steady_clock|high_resolution_clock)'
    )
    violations = []
    for source in sorted(_application_source_closure()):
        text = source.read_text(encoding='utf-8')
        if forbidden_includes.search(text) or forbidden_symbols.search(
                _without_comments(text)):
            violations.append(str(source.relative_to(PACKAGE_ROOT)))
    assert violations == []


def test_application_target_does_not_link_external_adapters():
    cmake = CMAKE.read_text(encoding='utf-8')
    start = cmake.index('add_library(graph_slam_application STATIC')
    end = cmake.index('add_library(graph_slam_io_adapters STATIC', start)
    target_definition = cmake[start:end]
    for forbidden in ('graph_slam_io_adapters', 'rclcpp', 'rosbag2',
                      'filesystem_io_ports'):
        assert forbidden not in target_definition


def test_live_and_offline_adapters_use_the_same_application_artifact_path():
    live = LIVE_ADAPTER_SOURCE.read_text(encoding='utf-8')
    offline = OFFLINE_ADAPTER_SOURCE.read_text(encoding='utf-8')
    for adapter in (live, offline):
        assert 'processSubmaps(' in adapter
        assert 'optimizeAndSerialize(' in adapter
        assert 'map_saver::loopEdgeCsvLine' not in adapter
        assert 'map_saver::trajectoryTumLine' not in adapter
