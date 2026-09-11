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

"""Tests for LiDAR-camera depth/image edge alignment metrics."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'evaluate_lidar_camera_alignment',
    REPO_ROOT / 'scripts' / 'evaluate_lidar_camera_alignment.py')
lca = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lca)


def test_image_edges_detects_vertical_step():
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[:, 10:] = 255
    edges = lca.image_edges(image, percentile=50)
    assert edges[:, 9:11].any()
    assert not edges[:, :5].any()


def test_depth_edges_detects_supported_depth_step():
    depth = np.full((10, 10), np.inf, dtype=np.float32)
    depth[5, 3:5] = [2.0, 4.0]
    edges = lca.depth_edges(depth, absolute=0.25, relative=0.0)
    assert edges[5, 3] and edges[5, 4]
    assert edges.sum() == 2


def test_depth_edge_normal_field_reports_horizontal_step_normal():
    depth = np.full((8, 8), np.inf, np.float32)
    depth[3, 2:6] = [2.0, 2.0, 4.0, 4.0]
    edges, nx, ny = lca.depth_edge_normal_field(
        depth, absolute=0.25, relative=0.0)
    assert edges[3, 3] and edges[3, 4]
    np.testing.assert_allclose(nx[3, 3:5], 1.0)
    np.testing.assert_allclose(ny[3, 3:5], 0.0)


def test_surface_supported_edges_keep_surfaces_and_reject_isolated_pair():
    depth = np.full((12, 12), np.inf, dtype=np.float32)
    depth[2:10, 2:6] = 2.0
    depth[2:10, 6:10] = 4.0
    depth[0, 0:2] = [2.0, 4.0]
    edges, raw_count = lca.supported_depth_edges(depth, {
        'radius': 1, 'min_neighbors': 2, 'absolute': 0.1, 'relative': 0.0,
    })
    assert raw_count == 18
    assert edges[:, 5:7].sum() == 16
    assert not edges[0, :2].any()


def test_surface_support_rejects_impossible_neighbour_count():
    with np.testing.assert_raises_regex(ValueError, 'exceed'):
        lca.surface_support_mask(
            np.ones((4, 4), np.float32), radius=1, min_neighbors=9)


def test_nearest_edge_distances_reports_alignment_and_shift():
    query = np.zeros((20, 20), dtype=bool)
    target = np.zeros_like(query)
    query[5:15, 8] = True
    target[5:15, 11] = True
    np.testing.assert_allclose(
        lca.nearest_edge_distances(query, target, max_distance=6), 3.0)
    np.testing.assert_allclose(
        lca.nearest_edge_distances(query, query, max_distance=6), 0.0)


def test_nearest_edge_correspondences_reports_signed_direction_and_saturation():
    query = np.zeros((12, 12), dtype=bool)
    target = np.zeros_like(query)
    query[4, 4] = True
    query[10, 10] = True
    target[2, 7] = True
    result = lca.nearest_edge_correspondences(query, target, max_distance=4)
    np.testing.assert_allclose(result['distance_px'][0], np.hypot(-2, 3))
    assert result['dy_px'][0] == -2
    assert result['dx_px'][0] == 3
    assert result['distance_px'][1] == 5
    assert np.isnan(result['dy_px'][1]) and np.isnan(result['dx_px'][1])


def test_oriented_correspondence_rejects_perpendicular_edge():
    query = np.zeros((12, 12), bool)
    target = np.zeros_like(query)
    query[6, 4] = True
    target[6, 6] = True
    qnx = np.zeros((12, 12), np.float32)
    qny = np.zeros_like(qnx)
    tnx = np.zeros_like(qnx)
    tny = np.zeros_like(qnx)
    qnx[6, 4] = 1.0
    tny[6, 6] = 1.0
    rejected = lca.nearest_oriented_edge_correspondences(
        query, target, qnx, qny, tnx, tny,
        max_distance=4, max_angle_deg=20.0)
    assert rejected['distance_px'][0] == 5.0
    tnx[6, 6], tny[6, 6] = 1.0, 0.0
    accepted = lca.nearest_oriented_edge_correspondences(
        query, target, qnx, qny, tnx, tny,
        max_distance=4, max_angle_deg=20.0)
    assert accepted['distance_px'][0] == 2.0


def test_projected_depth_keeps_nearest_point():
    points = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 5.0]])
    K = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    depth = lca.projected_depth(points, np.eye(4), K, 10, 10)
    assert depth[5, 5] == 2.0


def test_projected_depth_and_ids_keeps_nearest_source_id():
    points = np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 2.0]])
    K = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    depth, point_ids = lca.projected_depth_and_ids(
        points, np.eye(4), K, 10, 10)
    assert depth[5, 5] == 2.0
    assert point_ids[5, 5] == 1
    assert point_ids[0, 0] == -1


def test_extract_fixed_contours_uses_full_density_edges_and_caps_points():
    width = height = 12
    K = np.array([[10.0, 0.0, 6.0], [0.0, 10.0, 6.0], [0.0, 0.0, 1.0]])
    points = []
    for v in range(2, 10):
        for u in range(2, 10):
            z = 2.0 if u < 6 else 4.0
            points.append([(u - 6.0) * z / 10.0,
                           (v - 6.0) * z / 10.0, z])
    image = np.zeros((height, width, 3), np.uint8)
    image[:, 6:] = 255
    banks, report = lca.extract_fixed_contours(
        np.asarray(points), np.asarray([np.eye(4)]), K, [image],
        edge_percentile=50.0, association_distance=2,
        max_points_per_view=5)
    assert banks[0].shape == (5, 5)
    np.testing.assert_allclose(np.linalg.norm(banks[0][:, 3:5], axis=1), 1.0)
    assert report['views'][0]['raw_depth_edge_pixels'] == 16
    assert report['views'][0]['image_associated_edge_points_before_cap'] > 5
    assert report['total_fixed_contour_points'] == 5


def test_score_view_handles_no_depth_edges():
    points = np.array([[0.0, 0.0, 2.0]])
    K = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    result = lca.score_view(
        points, np.eye(4), K, np.zeros((10, 10, 3), dtype=np.uint8))
    assert result['edge_points'] == 0
    assert result['median_px'] is None
    assert result['out_of_range_fraction'] is None


def test_alignment_objective_uses_fixed_contours_without_dense_points():
    image = np.zeros((12, 12, 3), np.uint8)
    image[:, 6:] = 255
    K = np.array([[10.0, 0.0, 6.0], [0.0, 10.0, 6.0], [0.0, 0.0, 1.0]])
    contours = [np.array([[0.0, 0.0, 2.0]])]
    loss, metrics = lca.alignment_objective(
        np.zeros((0, 3)), np.asarray([np.eye(4)]), K, [image],
        np.zeros(6), edge_percentile=50.0,
        contour_points_by_view=contours)
    assert np.isfinite(loss)
    assert metrics['edge_points'] == 1
    assert metrics['raw_edge_points'] == 1


def test_alignment_objective_rejects_mismatched_contour_banks():
    with np.testing.assert_raises_regex(ValueError, 'bank count'):
        lca.alignment_objective(
            np.zeros((0, 3)), np.asarray([np.eye(4)]), np.eye(3),
            [np.zeros((4, 4), np.uint8)], np.zeros(6),
            contour_points_by_view=[])


def test_score_view_reports_search_range_saturation(monkeypatch):
    monkeypatch.setattr(
        lca, 'nearest_edge_correspondences',
        lambda *args, **kwargs: {
            'distance_px': np.array([0.0, 13.0], dtype=np.float32),
            'dx_px': np.array([0.0, np.nan], dtype=np.float32),
            'dy_px': np.array([0.0, np.nan], dtype=np.float32)})
    result = lca.score_view(
        np.array([[0.0, 0.0, 2.0]]), np.eye(4), np.eye(3),
        np.zeros((10, 10, 3), dtype=np.uint8), max_distance=12)
    assert result['out_of_range_fraction'] == 0.5
    assert result['matched_edge_points'] == 1


def test_write_residual_diagnostics_ranks_worst_view_and_writes_pngs(
        tmp_path, monkeypatch):
    overlays = []

    def render(*args, **kwargs):
        value = len(overlays) + 1
        overlays.append(value)
        return np.full((8, 10, 3), value, dtype=np.uint8)

    monkeypatch.setattr(lca, 'render_residual_overlay', render)
    per_view = [
        {'view_index': 0, 'edge_points': 10, 'matched_edge_points': 8,
         'median_px': 2.0, 'p90_px': 4.0, 'inlier_2px': 0.5,
         'out_of_range_fraction': 0.1, 'median_dx_px': 1.0,
         'median_dy_px': 0.0, 'mean_dx_px': 1.0, 'mean_dy_px': 0.0,
         'direction_coherence': 0.5},
        {'view_index': 1, 'edge_points': 10, 'matched_edge_points': 5,
         'median_px': 8.0, 'p90_px': 13.0, 'inlier_2px': 0.1,
         'out_of_range_fraction': 0.5, 'median_dx_px': -2.0,
         'median_dy_px': 3.0, 'mean_dx_px': -2.0, 'mean_dy_px': 3.0,
         'direction_coherence': 0.8},
    ]
    result = lca.write_residual_diagnostics(
        tmp_path, np.zeros((0, 3)), np.asarray([np.eye(4), np.eye(4)]),
        np.eye(3), [0, 1], [np.zeros((8, 10, 3), np.uint8)] * 2,
        per_view, worst_views=1, edge_percentile=95.0, max_distance=12)
    assert result['worst_views'][0]['view_index'] == 1
    assert (tmp_path / 'worst_01_view_00001.png').is_file()
    assert (tmp_path / 'worst_views_contact_sheet.png').is_file()
    assert json.loads((tmp_path / 'diagnostics.json').read_text())[
        'direction_summary']['weighted_mean_dx_px'] == -2 / 13


def test_correction_matrix_applies_translation_and_rotation():
    matrix = lca.correction_matrix([1.0, 2.0, 3.0, 0.0, 0.0, np.pi / 2])
    np.testing.assert_allclose(matrix[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(
        matrix[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_optimize_correction_descends_synthetic_objective(monkeypatch):
    target = np.array([0.02, -0.01, 0.0, np.deg2rad(0.2), 0.0, 0.0])

    def objective(*args, reference_edge_points=None, **kwargs):
        parameters = args[4]
        loss = float(np.sum((parameters - target) ** 2))
        return loss, {'edge_points': 100, 'mean_px': loss,
                      'median_px': loss, 'coverage': 1.0}

    monkeypatch.setattr(lca, 'alignment_objective', objective)
    parameters, before, after = lca.optimize_correction(
        np.zeros((0, 3)), np.zeros((0, 4, 4)), np.eye(3), [], rounds=2)
    assert after['loss'] < before['loss']
    np.testing.assert_allclose(parameters, target, atol=np.deg2rad(0.1))


def test_write_corrected_transforms_preserves_images_and_updates_pose(tmp_path):
    images = tmp_path / 'source' / 'images'
    images.mkdir(parents=True)
    (images / '000.png').write_bytes(b'pixel')
    source = tmp_path / 'source' / 'transforms.json'
    source.write_text(json.dumps({
        'fl_x': 10, 'fl_y': 10, 'cx': 5, 'cy': 5, 'w': 10, 'h': 10,
        'frames': [{'file_path': 'images/000.png',
                    'transform_matrix': np.eye(4).tolist()}],
    }))
    output = tmp_path / 'result' / 'corrected.json'
    correction = lca.correction_matrix([0.1, 0, 0, 0, 0, 0])
    original = lca.tg.load_transforms(source)['viewmats'][0]
    lca.write_corrected_transforms(source, output, correction)
    loaded = lca.tg.load_transforms(output)
    np.testing.assert_allclose(loaded['viewmats'][0], correction @ original)
    assert loaded['image_paths'][0] == (images / '000.png').resolve()


def test_write_recomposed_transforms_embeds_calibration_uncertainty(tmp_path):
    images = tmp_path / 'source' / 'images'
    images.mkdir(parents=True)
    (images / '000.png').write_bytes(b'pixel')
    source = tmp_path / 'source' / 'transforms.json'
    source.write_text(json.dumps({
        'fl_x': 10, 'fl_y': 10, 'cx': 5, 'cy': 5, 'w': 10, 'h': 10,
        'frames': [{'file_path': 'images/000.png',
                    'transform_matrix': np.eye(4).tolist()}],
    }))
    output = tmp_path / 'result' / 'recomposed.json'
    calibration = {
        'accepted': True,
        'uncertainty_dt_s_xyz_m_rpy_rad': [0.01] * 7,
    }
    lca.write_recomposed_transforms(
        source, output, np.asarray([np.eye(4)]),
        calibration=calibration)
    document = json.loads(output.read_text())
    assert document['spatiotemporal_calibration'] == calibration
    assert (lca.tg.load_transforms(output)['image_paths'][0] ==
            (images / '000.png').resolve())


def test_calibration_metadata_exposes_compact_fusion_contract():
    observability = {
        'uncertainty_dt_s_xyz_m_rpy_rad': [0.01] * 7,
        'condition_number': 2.0,
        'maximum_abs_time_translation_correlation': 0.1,
    }
    metadata = lca.calibration_metadata({
        'accepted': True,
        'parameters_dt_s_xyz_m_rpy_deg': [0.0] * 7,
        'boundary_axes': [],
        'production_calibration': {'observability': observability},
    })
    assert metadata['accepted']
    assert metadata['uncertainty_dt_s_xyz_m_rpy_rad'] == [0.01] * 7
    assert metadata['condition_number'] == 2.0


def _moving_samples():
    return [
        lca.pi.TrajectorySample(
            float(index), np.array([float(index), 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]))
        for index in range(4)
    ]


def test_recompose_viewmats_applies_continuous_time_and_extrinsic_delta():
    parameters = np.array([0.25, 0.1, 0, 0, 0, 0, 0])
    viewmat = lca.recompose_viewmats(
        _moving_samples(), np.array([1.0]), np.eye(4), parameters)[0]
    # world<-camera x is trajectory(1.25) + local extrinsic x correction.
    np.testing.assert_allclose(np.linalg.inv(viewmat)[:3, 3], [1.35, 0, 0])


def test_infer_body_camera_recovers_static_extrinsic_and_consistency():
    samples = _moving_samples()
    stamps = np.array([0.0, 1.0, 2.0])
    extrinsic = lca.correction_matrix([0.1, -0.2, 0.3, 0.01, 0.02, -0.03])
    viewmats = np.asarray([
        np.linalg.inv(lca.pi.interpolate_pose(samples, stamp) @ extrinsic)
        for stamp in stamps])
    inferred, consistency = lca.infer_body_T_camera(
        samples, stamps, viewmats)
    np.testing.assert_allclose(inferred, extrinsic, atol=1e-12)
    assert consistency['translation_spread_p95_m'] < 1e-12
    assert consistency['rotation_spread_p95_deg'] < 1e-5


def test_spatiotemporal_optimizer_is_deterministic_and_bounded(monkeypatch):
    target = np.array([0.08, 0.08, -0.04, 0.0,
                       np.deg2rad(0.4), 0.0, 0.0])

    def objective(*args, reference_edge_points=None, **kwargs):
        parameters = np.asarray(args[6])
        loss = float(np.sum((parameters - target) ** 2))
        return loss, {'edge_points': 100, 'mean_px': loss,
                      'median_px': loss, 'coverage': 1.0}

    monkeypatch.setattr(lca, 'spatiotemporal_objective', objective)

    def call():
        return lca.optimize_spatiotemporal(
            np.zeros((0, 3)), _moving_samples(), np.array([1.0]), np.eye(4),
            np.eye(3), [], rounds=3, time_step=0.04,
            translation_step=0.04, rotation_step_deg=0.4,
            max_time_offset=0.05, max_translation=0.05,
            max_rotation_deg=0.3)
    first, before, after = call()
    second, _, _ = call()
    np.testing.assert_array_equal(first, second)
    assert after['loss'] < before['loss']
    assert abs(first[0]) <= 0.05
    assert np.all(np.abs(first[1:4]) <= 0.05)
    assert np.all(np.abs(np.rad2deg(first[4:])) <= 0.3 + 1e-12)


def test_production_optimizer_runs_pyramid_and_observability(monkeypatch):
    target = np.array([0.02, -0.02, 0.0, 0.0,
                       np.deg2rad(0.2), 0.0, 0.0])

    def objective(*args, reference_edge_points=None,
                  image_edge_masks=None, **kwargs):
        parameters = np.asarray(args[6])
        loss = 1.0 + float(np.sum((parameters - target) ** 2))
        return loss, {'edge_points': 100, 'mean_px': loss,
                      'median_px': loss, 'coverage': 1.0}

    monkeypatch.setattr(lca, 'spatiotemporal_objective', objective)
    parameters, before, after, report = \
        lca.optimize_spatiotemporal_production(
            np.zeros((0, 3)), _moving_samples(), np.array([1.0]),
            np.eye(4), np.eye(3), [np.zeros((20, 20), np.uint8)],
            scales=(0.5, 1.0), rounds_per_level=2,
            time_step=0.02, translation_step=0.02,
            rotation_step_deg=0.2, max_time_offset=0.1,
            max_translation=0.1, max_rotation_deg=1.0,
            auto_bound_expansions=0, minimum_curvature=1e-12)
    assert after['loss'] < before['loss']
    np.testing.assert_allclose(parameters, target, atol=0.005)
    assert [level['scale'] for level in report['levels']] == [0.5, 1.0]
    assert report['observability']['observable']
    assert not report['boundary_axes']


def test_trajectory_excitation_rejects_static_time_offset():
    static = [
        lca.pi.TrajectorySample(
            float(index), np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
        for index in range(3)
    ]
    assert not lca.trajectory_excitation(
        static, np.array([0.0, 1.0, 2.0]))['time_offset_observable']
    assert lca.trajectory_excitation(
        _moving_samples(), np.array([0.0, 1.0, 2.0]))['time_offset_observable']


def test_calibration_acceptance_requires_heldout_improvement_and_support():
    before = {'edge_points': 100, 'loss': 10.0}
    improved = {'edge_points': 100, 'loss': 8.0}
    accepted, reason = lca.calibration_acceptance(
        before, improved, before, improved,
        minimum_edge_points=50, minimum_heldout_improvement=0.1)
    assert accepted and reason is None
    accepted, reason = lca.calibration_acceptance(
        before, improved, before, {'edge_points': 100, 'loss': 9.5},
        minimum_edge_points=50, minimum_heldout_improvement=0.1)
    assert not accepted and reason == 'heldout_or_training_loss_did_not_improve'
    accepted, _ = lca.calibration_acceptance(
        before, improved, before, before,
        minimum_edge_points=50, minimum_heldout_improvement=0.0)
    assert not accepted
    accepted, reason = lca.calibration_acceptance(
        {'edge_points': 10, 'loss': 10.0}, improved, before, improved,
        minimum_edge_points=50, minimum_heldout_improvement=0.1)
    assert not accepted and reason == 'insufficient_edge_support'
    filtered = {'edge_points': 100, 'loss': 10.0,
                'supported_edge_fraction': 0.08}
    accepted, reason = lca.calibration_acceptance(
        filtered, improved, filtered, improved,
        minimum_edge_points=50, minimum_heldout_improvement=0.1,
        minimum_supported_edge_fraction=0.25)
    assert not accepted and reason == 'insufficient_supported_edge_fraction'
