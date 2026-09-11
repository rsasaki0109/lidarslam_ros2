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

"""Regression tests for triangle descriptor parameters in default YAML files."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

EXPECTED_PARAM_KEYS = {
    'use_triangle_descriptor',
    'triangle_descriptor_keypoint_mode',
    'triangle_descriptor_grid_size_m',
    'triangle_descriptor_grid_cells',
    'triangle_descriptor_max_keypoints',
    'triangle_descriptor_min_salience_m',
    'triangle_descriptor_edge_voxel_size_m',
    'triangle_descriptor_edge_neighbor_radius_m',
    'triangle_descriptor_edge_min_neighbors',
    'triangle_descriptor_edge_min_edgeness',
    'triangle_descriptor_edge_nms_radius_m',
    'triangle_descriptor_min_edge_m',
    'triangle_descriptor_max_edge_m',
    'triangle_descriptor_max_triangles',
    'triangle_descriptor_edge_bin_m',
    'triangle_descriptor_quad_feature_bin_m',
    'triangle_descriptor_min_votes',
    'triangle_descriptor_min_inliers',
    'triangle_descriptor_inlier_translation_m',
    'triangle_descriptor_inlier_rotation_deg',
    'triangle_descriptor_exclude_recent',
    'triangle_descriptor_min_inlier_ratio',
    'triangle_descriptor_max_pairs',
    'triangle_descriptor_min_4th_point_agreements',
    'triangle_descriptor_fourth_point_max_distance_m',
    'triangle_descriptor_refine_se3_with_all_inliers',
    'triangle_verify_with_bev',
    'triangle_verify_bev_max_distance',
}

VALID_KEYPOINT_MODES = {'bev_max_height', 'edge_3d'}

PARAM_FILES = [
    REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam.yaml',
    REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam_indoor.yaml',
    REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml',
]

INDOOR_PARAM_FILE = REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam_indoor.yaml'
MID360_PARAM_FILE = REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'
GENERIC_PARAM_FILE = REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam.yaml'


def _load_graph_params(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert 'graph_based_slam' in data, f'{path} missing graph_based_slam node'
    params = data['graph_based_slam'].get('ros__parameters')
    assert params is not None, f'{path} missing ros__parameters'
    return params


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_yaml_contains_all_triangle_descriptor_keys(path):
    params = _load_graph_params(path)
    missing = EXPECTED_PARAM_KEYS - set(params.keys())
    assert not missing, (
        f'{path.name} missing keys: {sorted(missing)}'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_triangle_descriptor_defaults_off(path):
    params = _load_graph_params(path)
    assert params['use_triangle_descriptor'] is False, (
        f'{path.name}: triangle descriptor must default off so the existing '
        'workflow stays unchanged'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_triangle_descriptor_edge_bounds_sane(path):
    params = _load_graph_params(path)
    min_edge = params['triangle_descriptor_min_edge_m']
    max_edge = params['triangle_descriptor_max_edge_m']
    assert min_edge > 0.0, f'{path.name}: min_edge_m must be positive'
    assert max_edge > min_edge, (
        f'{path.name}: max_edge_m ({max_edge}) must exceed min_edge_m ({min_edge})'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_triangle_descriptor_quad_hash_default_disabled(path):
    """
    Quad-hash extension must default to 0 so the legacy 3-edge hash holds.

    The packed hash key is bit-for-bit identical when quad_feature_bin_m == 0,
    so opting in is the only way to change bucket assignments. Negative is
    rejected at runtime; here we just guard the yaml against accidental
    negative defaults.
    """
    params = _load_graph_params(path)
    bin_m = params['triangle_descriptor_quad_feature_bin_m']
    assert bin_m >= 0.0, (
        f'{path.name}: quad_feature_bin_m must be >= 0 (0 = disabled); got {bin_m}'
    )


def test_mid360_preset_tightens_for_short_range_sensor():
    """MID-360 has shorter range than spinning 360° LiDAR; preset must reflect."""
    default = _load_graph_params(GENERIC_PARAM_FILE)
    mid360 = _load_graph_params(MID360_PARAM_FILE)
    assert mid360['triangle_descriptor_max_edge_m'] <= default['triangle_descriptor_max_edge_m'], (
        'MID-360 preset must not exceed the generic LiDAR max edge (shorter range sensor)'
    )
    assert mid360['triangle_descriptor_min_votes'] >= default['triangle_descriptor_min_votes'], (
        'MID-360 preset should be stricter on vote count to suppress FOV ambiguity'
    )


def test_indoor_preset_uses_tighter_edge3d():
    """
    Indoor preset must encode the tighter edge_3d + 4-point gate recipe.

    Newer College math_hard ablation (2026-05-19) showed that tighter voxel /
    smaller neighbor radius / higher edgeness produces the first inliers=4 emit
    and APE -0.039m (past max). See docs/research/project-plan-archive.md §1.2 and
    project_triangle_descriptor_stack memory.
    """
    indoor = _load_graph_params(INDOOR_PARAM_FILE)
    assert indoor['triangle_descriptor_keypoint_mode'] == 'edge_3d', (
        'Indoor preset must default to edge_3d; BEV max-height fails in indoor scenes'
    )
    assert indoor['triangle_descriptor_edge_voxel_size_m'] <= 0.25, (
        'Indoor preset must use tighter voxel (<=0.25 m); 0.4 m default is too coarse'
    )
    assert indoor['triangle_descriptor_edge_neighbor_radius_m'] <= 0.7, (
        'Indoor preset must use tighter neighbor radius (<=0.7 m)'
    )
    assert indoor['triangle_descriptor_edge_min_edgeness'] >= 0.55, (
        'Indoor preset must use stricter edgeness floor (>=0.55) to suppress weak PCA cases'
    )
    assert indoor['triangle_descriptor_min_4th_point_agreements'] >= 2, (
        'Indoor preset must enable 4-point gate (>=2) to filter yaw-flip false positives'
    )
    assert indoor['triangle_descriptor_min_inliers'] <= 3, (
        'Indoor preset must keep min_inliers <= 3 so the edge_3d inlier shift can reach emit'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_bev_cross_verify_defaults_off(path):
    """Cross-verification must be opt-in so default workflows stay unchanged."""
    params = _load_graph_params(path)
    assert params['triangle_verify_with_bev'] is False
    assert params['triangle_verify_bev_max_distance'] > 0.0


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_keypoint_mode_is_valid(path):
    params = _load_graph_params(path)
    mode = params['triangle_descriptor_keypoint_mode']
    assert mode in VALID_KEYPOINT_MODES, (
        f'{path.name}: unknown keypoint mode {mode!r}; '
        f'must be one of {sorted(VALID_KEYPOINT_MODES)}'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_edge_keypoint_params_sane(path):
    params = _load_graph_params(path)
    assert params['triangle_descriptor_edge_voxel_size_m'] >= 0.0
    assert params['triangle_descriptor_edge_neighbor_radius_m'] > 0.0
    assert params['triangle_descriptor_edge_min_neighbors'] >= 4
    edgeness = params['triangle_descriptor_edge_min_edgeness']
    assert 0.0 <= edgeness <= 1.0, (
        f'{path.name}: edgeness threshold {edgeness} out of [0, 1]'
    )
    assert params['triangle_descriptor_edge_nms_radius_m'] >= 0.0


def test_mid360_preset_prefers_edge_keypoints():
    """
    MID-360 preset must default to edge_3d keypoints.

    MID-360 narrow-FOV ablation showed BEV max-height fails (votes accumulate
    but 3-point RANSAC inliers stay at 1-2). edge_3d is the cross-dataset
    answer (see project_triangle_descriptor_stack memory).
    """
    mid360 = _load_graph_params(MID360_PARAM_FILE)
    assert mid360['triangle_descriptor_keypoint_mode'] == 'edge_3d', (
        'MID-360 preset must default to edge_3d so the opt-in triangle path '
        'has a working keypoint extractor on narrow-FOV LiDAR'
    )


def test_indoor_preset_voxel_tighter_than_mid360():
    """
    Indoor preset must use a tighter edge_3d voxel than the MID-360 preset.

    Newer College ablation (2026-05-19) showed indoor scenes benefit from a
    tighter voxel than the MID-360 outdoor preset (0.2 m vs 0.3 m).
    """
    indoor = _load_graph_params(INDOOR_PARAM_FILE)
    mid360 = _load_graph_params(MID360_PARAM_FILE)
    assert indoor['triangle_descriptor_edge_voxel_size_m'] \
        < mid360['triangle_descriptor_edge_voxel_size_m'], (
        'Indoor preset must use a tighter voxel than the MID-360 outdoor preset'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_loop_max_delta_descriptor_override_present_and_safe(path):
    """
    Descriptor-only loop_max_delta override must be present and safe by default.

    Defaults to -1 (disabled) so opting in is explicit per dataset.
    DISTANCE candidates keep the strict generic cap; descriptor sources can
    accept a larger NDT correction when the operator sets a positive value.
    The default-disabled stance keeps backward compatibility — every yaml
    that shipped before the override was added used a single cap.
    """
    params = _load_graph_params(path)
    assert 'loop_max_translation_delta_descriptor' in params, (
        f'{path.name}: missing loop_max_translation_delta_descriptor — the '
        'override is the documented escape hatch for long-baseline triangle '
        'loops; ship it (even at -1) so users know it exists'
    )
    assert 'loop_max_rotation_delta_deg_descriptor' in params
    t_override = params['loop_max_translation_delta_descriptor']
    r_override = params['loop_max_rotation_delta_deg_descriptor']
    assert t_override == -1.0 or t_override > 0.0, (
        f'{path.name}: loop_max_translation_delta_descriptor must be -1 '
        f'(disabled) or > 0; got {t_override}'
    )
    assert r_override == -1.0 or r_override > 0.0, (
        f'{path.name}: loop_max_rotation_delta_deg_descriptor must be -1 '
        f'(disabled) or > 0; got {r_override}'
    )
