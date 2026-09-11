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

"""Tests for the user-facing coloured-map pipeline command composition."""

import os
from pathlib import Path
import sys

import imageio as iio
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import colored_map_pipeline as cmp  # noqa: E402, I100


def _write_at(path, text, stamp_ns):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    os.utime(path, ns=(stamp_ns, stamp_ns))


def _args(tmp_path, *extra):
    return cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--extrinsic', str(tmp_path / 'calib.json'), *extra,
    ])


def test_build_commands_connects_extract_to_robust_map(tmp_path):
    commands = cmp.build_commands(_args(tmp_path))
    assert [name for name, _ in commands] == ['posed images', 'coloured map']
    extract, build = commands[0][1], commands[1][1]
    transforms = str(tmp_path / 'out' / 'posed_images' / 'transforms.json')
    assert '--undistort' in extract
    assert extract[extract.index('--time-offset') + 1] == 'auto'
    assert extract[extract.index('--time-offset-adjustment') + 1] == '0.0'
    assert build[build.index('--color-transforms') + 1] == transforms
    assert '--color-robust' in build
    assert build[build.index('--min-neighbors') + 1] == '2'
    assert build[build.index('--sparse-voxel') + 1] == '0.1'
    assert build[build.index('--color-exposure-scale-limit') + 1] == '1.5'
    assert build[build.index('--color-max-samples') + 1] == '12'
    assert '--color-no-normalize-exposure' not in build


def test_colour_ablation_options_reach_map_builder(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--color-no-normalize-exposure',
        '--color-exposure-scale-limit', '1.2', '--color-max-samples', '5'))
    build = commands[1][1]
    assert '--color-no-normalize-exposure' in build
    assert build[build.index('--color-exposure-scale-limit') + 1] == '1.2'
    assert build[build.index('--color-max-samples') + 1] == '5'


def test_time_offset_adjustment_reaches_image_extractor(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--time-offset-adjustment', '-0.02'))
    extract = commands[0][1]
    assert extract[extract.index('--time-offset-adjustment') + 1] == '-0.02'


def test_spatiotemporal_refinement_inserts_geometry_and_heldout_calibration(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--refine-spatiotemporal-calibration',
        '--calibration-max-time-offset', '0.04',
        '--calibration-minimum-heldout-improvement', '0.02'))
    assert [name for name, _ in commands] == [
        'posed images', 'calibration geometry',
        'spatiotemporal calibration', 'coloured map']
    geometry = dict(commands)['calibration geometry']
    calibration = dict(commands)['spatiotemporal calibration']
    coloured = dict(commands)['coloured map']
    assert '--color-transforms' not in geometry
    assert '--optimize-spatiotemporal' in calibration
    assert '--production-calibration' in calibration
    assert calibration[calibration.index('--max-time-offset') + 1] == '0.04'
    assert calibration[calibration.index('--max-points') + 1] == '300000'
    assert calibration[calibration.index('--pyramid-scales') + 1] == \
        '0.25,0.5,1.0'
    assert calibration[calibration.index('--holdout-fraction') + 1] == '0.2'
    assert calibration[
        calibration.index('--minimum-heldout-improvement') + 1] == '0.02'
    refined = str(
        tmp_path / 'out' / 'posed_images' /
        'transforms_spatiotemporal.json')
    assert calibration[
        calibration.index('--corrected-transforms-out') + 1] == refined
    assert coloured[coloured.index('--color-transforms') + 1] == refined


def test_geometry_aware_fusion_forwards_all_production_guards(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--refine-spatiotemporal-calibration',
        '--color-geometry-aware', '--color-occlusion-margin-px', '3',
        '--color-depth-edge-margin-px', '4',
        '--dynamic-mask-dir', str(tmp_path / 'masks'),
        '--color-dynamic-exclusion', '--color-dynamic-mask-margin-px', '5',
        '--color-calibration-sigma-multiplier', '1.5',
        '--color-max-uncertainty-margin-px', '9'))
    coloured = dict(commands)['coloured map']
    assert '--color-geometry-aware' in coloured
    assert coloured[coloured.index('--color-occlusion-margin-px') + 1] == '3'
    assert coloured[coloured.index('--color-depth-edge-margin-px') + 1] == '4'
    assert '--color-dynamic-exclusion' in coloured
    assert coloured[coloured.index('--color-dynamic-mask-margin-px') + 1] == '5'
    assert coloured[
        coloured.index('--color-calibration-sigma-multiplier') + 1] == '1.5'
    assert coloured[
        coloured.index('--color-max-uncertainty-margin-px') + 1] == '9'
    attach = dict(commands)['dynamic image masks']
    masked = str(tmp_path / 'out' / 'posed_images' /
                 'transforms_dynamic_masks.json')
    calibration = dict(commands)['spatiotemporal calibration']
    assert attach[attach.index('--mask-dir') + 1] == str(tmp_path / 'masks')
    assert attach[attach.index('--out') + 1] == masked
    assert calibration[calibration.index('--transforms') + 1] == masked


def test_geometry_boundary_guards_are_default_off(tmp_path):
    args = _args(tmp_path, '--color-geometry-aware')
    coloured = dict(cmp.build_commands(args))['coloured map']
    assert coloured[coloured.index('--color-occlusion-margin-px') + 1] == '0'
    assert coloured[coloured.index('--color-depth-edge-margin-px') + 1] == '0'


def test_dynamic_masks_are_cached_and_new_masks_rebuild_dependents(tmp_path):
    out = tmp_path / 'out'
    masks = tmp_path / 'masks'
    masks.mkdir()
    _write_at(tmp_path / 'traj.tum', 'dense\n', 1)
    _write_at(out / 'posed_images' / 'transforms.json', '{}', 2)
    _write_at(masks / 'frame.png', 'mask', 2)
    os.utime(masks, ns=(2, 2))
    _write_at(out / 'posed_images' / 'transforms_dynamic_masks.json', '{}', 3)
    _write_at(out / 'colored_map.ply', 'ply\n', 4)
    args = _args(tmp_path, '--dynamic-mask-dir', str(masks))
    assert cmp.build_commands(args) == []
    os.utime(masks / 'frame.png', ns=(5, 5))
    assert [name for name, _ in cmp.build_commands(args)] == [
        'dynamic image masks', 'coloured map']


def test_dynamic_exclusion_requires_mask_dataset(tmp_path):
    import pytest
    args = _args(tmp_path, '--color-geometry-aware',
                 '--color-dynamic-exclusion', '--dry-run')
    with pytest.raises(ValueError, match='dynamic-mask-dir'):
        cmp.run_pipeline(args)


def test_existing_spatiotemporal_outputs_are_reused(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'spatiotemporal_calibration_geometry.ply').write_text('ply\n')
    (out / 'posed_images' / 'transforms_spatiotemporal.json').write_text('{}')
    (out / 'spatiotemporal_calibration.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    assert cmp.build_commands(_args(
        tmp_path, '--refine-spatiotemporal-calibration')) == []


def test_force_calibration_requires_opt_in(tmp_path):
    import pytest
    with pytest.raises(ValueError, match='refine-spatiotemporal'):
        cmp.run_pipeline(_args(tmp_path, '--force-calibration', '--dry-run'))


def test_raw_trajectory_adds_densification_and_connects_dense_output(tmp_path):
    args = _args(tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'))
    commands = cmp.build_commands(args)
    assert [name for name, _ in commands] == [
        'dense corrected trajectory', 'posed images', 'coloured map']
    densify, extract, build = [command for _, command in commands]
    dense = str(tmp_path / 'out' / 'dense_corrected_trajectory.tum')
    assert densify[densify.index('--raw') + 1] == str(tmp_path / 'raw.tum')
    assert densify[densify.index('--corrected') + 1] == str(
        tmp_path / 'traj.tum')
    assert densify[densify.index('--output') + 1] == dense
    assert extract[extract.index('--traj') + 1] == dense
    assert build[build.index('--traj') + 1] == dense


def test_existing_dense_trajectory_and_outputs_are_reused(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'dense_corrected_trajectory.tum').write_text('dense\n')
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    args = _args(tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'))
    assert cmp.build_commands(args) == []


def test_force_trajectory_rebuilds_all_dependent_stages(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'dense_corrected_trajectory.tum').write_text('dense\n')
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    args = _args(
        tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'),
        '--force-trajectory')
    assert [name for name, _ in cmp.build_commands(args)] == [
        'dense corrected trajectory', 'posed images', 'coloured map']


def test_newer_trajectory_input_rebuilds_all_dependent_stages(tmp_path):
    out = tmp_path / 'out'
    _write_at(out / 'dense_corrected_trajectory.tum', 'dense\n', 2)
    _write_at(out / 'posed_images' / 'transforms.json', '{}', 3)
    _write_at(out / 'colored_map.ply', 'ply\n', 4)
    _write_at(tmp_path / 'raw.tum', 'raw\n', 5)
    _write_at(tmp_path / 'traj.tum', 'corrected\n', 1)
    args = _args(tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'))
    assert [name for name, _ in cmp.build_commands(args)] == [
        'dense corrected trajectory', 'posed images', 'coloured map']


def test_newer_direct_trajectory_rebuilds_images_and_map(tmp_path):
    out = tmp_path / 'out'
    _write_at(out / 'posed_images' / 'transforms.json', '{}', 2)
    _write_at(out / 'colored_map.ply', 'ply\n', 3)
    _write_at(tmp_path / 'traj.tum', 'dense\n', 4)
    assert [name for name, _ in cmp.build_commands(_args(tmp_path))] == [
        'posed images', 'coloured map']


def test_newer_posed_images_rebuild_only_map(tmp_path):
    out = tmp_path / 'out'
    _write_at(tmp_path / 'traj.tum', 'dense\n', 1)
    _write_at(out / 'colored_map.ply', 'ply\n', 2)
    _write_at(out / 'posed_images' / 'transforms.json', '{}', 3)
    assert [name for name, _ in cmp.build_commands(_args(tmp_path))] == [
        'coloured map']


def test_build_commands_reuses_existing_outputs(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    assert cmp.build_commands(_args(tmp_path)) == []


def test_force_map_reuses_images_but_rebuilds_map(tmp_path):
    posed = tmp_path / 'out' / 'posed_images'
    posed.mkdir(parents=True)
    (posed / 'transforms.json').write_text('{}')
    commands = cmp.build_commands(_args(tmp_path, '--force-map'))
    assert [name for name, _ in commands] == ['coloured map']


def test_force_images_also_rebuilds_dependent_map(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    commands = cmp.build_commands(_args(tmp_path, '--force-images'))
    assert [name for name, _ in commands] == ['posed images', 'coloured map']


def test_no_undistort_and_custom_topics_are_forwarded(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--no-undistort', '--points-topic', '/points',
        '--camera-topic', '/rgb', '--camera-info-topic', '/info'))
    extract, build = commands[0][1], commands[1][1]
    assert '--undistort' not in extract
    assert extract[extract.index('--camera-topic') + 1] == '/rgb'
    assert extract[extract.index('--camera-info-topic') + 1] == '/info'
    assert build[build.index('--points-topic') + 1] == '/points'


def test_custom_density_filter_is_forwarded(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--min-neighbors', '0', '--sparse-voxel', '0.2'))
    build = commands[-1][1]
    assert build[build.index('--min-neighbors') + 1] == '0'
    assert build[build.index('--sparse-voxel') + 1] == '0.2'


def test_no_deskew_is_forwarded_to_map_builder(tmp_path):
    commands = cmp.build_commands(_args(tmp_path, '--no-deskew'))

    assert '--no-deskew' in commands[-1][1]


def test_intrinsics_yaml_is_forwarded_for_bags_without_camera_info(tmp_path):
    intrinsics = tmp_path / 'camchain.yaml'
    commands = cmp.build_commands(_args(
        tmp_path, '--intrinsics-yaml', str(intrinsics)))
    extract = commands[0][1]
    assert extract[extract.index('--intrinsics-yaml') + 1] == str(intrinsics)


def test_kalibr_pair_uses_generated_extrinsic(tmp_path):
    args = cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--kalibr-camchain', str(tmp_path / 'camchain.yaml'),
        '--lidar-calibration', str(tmp_path / 'lidar.yaml'),
    ])
    extract, build = [command for _, command in cmp.build_commands(args)]
    generated = tmp_path / 'out' / 'generated_body_camera_extrinsic.json'
    assert extract[extract.index('--extrinsic') + 1] == str(generated)
    assert build[build.index('--lidar-calibration') + 1] == str(
        tmp_path / 'lidar.yaml')
    assert build[build.index('--lidar-key') + 1] == 'PandarXT-32'


def test_kalibr_camchain_requires_lidar_calibration(tmp_path):
    import pytest
    args = cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--kalibr-camchain', str(tmp_path / 'camchain.yaml'), '--dry-run',
    ])
    with pytest.raises(ValueError, match='lidar-calibration'):
        cmp.run_pipeline(args)


def test_sparse_graph_keyframes_are_rejected(tmp_path):
    import pytest
    traj = tmp_path / 'traj.tum'
    traj.write_text(
        '1.0 0 0 0 0 0 0 1\n'
        '2.2 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='dense SLAM trajectory'):
        cmp.validate_trajectory_density(traj, 0.5)


def test_dense_trajectory_passes_density_guard(tmp_path):
    traj = tmp_path / 'traj.tum'
    traj.write_text(
        '1.0 0 0 0 0 0 0 1\n'
        '1.1 0 0 0 0 0 0 1\n')
    cmp.validate_trajectory_density(traj, 0.5)


def test_colour_source_rejects_mono_and_accepts_real_rgb(tmp_path):
    images = tmp_path / 'images'
    images.mkdir()
    iio.imwrite(images / 'mono.png', np.zeros((8, 8), np.uint8))
    transforms = tmp_path / 'transforms.json'
    transforms.write_text(
        '{"frames": [{"file_path": "images/mono.png"}]}')
    import pytest
    with pytest.raises(ValueError, match='monochrome'):
        cmp.validate_colour_source(transforms)
    assert not cmp.validate_colour_source(
        transforms, allow_monochrome=True)['is_colour']

    rgb = np.zeros((8, 8, 3), np.uint8)
    rgb[:, :, 0] = 200
    iio.imwrite(images / 'rgb.png', rgb)
    transforms.write_text(
        '{"frames": [{"file_path": "images/rgb.png"}]}')
    assert cmp.validate_colour_source(transforms)['is_colour']


def test_quality_profile_adds_two_evaluators_and_gate(tmp_path):
    args = _args(
        tmp_path, '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--trajectory-report', str(tmp_path / 'metrics.json'),
        '--geometry-report', str(tmp_path / 'map_quality_report.yaml'))
    commands = cmp.build_commands(args)
    assert [name for name, _ in commands][-4:] == [
        'camera-LiDAR alignment', 'held-out colour', 'appearance',
        'quality gate']
    gate = commands[-1][1]
    assert gate[gate.index('--trajectory-report') + 1] == str(
        tmp_path / 'metrics.json')
    assert gate[gate.index('--geometry-report') + 1] == str(
        tmp_path / 'map_quality_report.yaml')
    assert gate[gate.index('--alignment-report') + 1].endswith(
        'out/lidar_camera_alignment.json')
    assert gate[gate.index('--colour-report') + 1].endswith(
        'out/heldout_point_colors.json')


def test_quality_profile_allows_appearance_and_colour_only(tmp_path):
    args = _args(
        tmp_path, '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--dry-run')
    commands = cmp.build_commands(args)
    gate = dict(commands)['quality gate']
    assert '--trajectory-report' not in gate
    assert '--geometry-report' not in gate
    assert '--alignment-report' in gate
    assert '--colour-report' in gate
    assert '--appearance-report' in gate


def test_quality_profile_can_emit_alignment_residual_diagnostics(tmp_path):
    args = _args(
        tmp_path, '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--alignment-diagnostics', '--alignment-diagnostic-worst-views', '6')
    alignment = dict(cmp.build_commands(args))['camera-LiDAR alignment']
    diagnostics = alignment[alignment.index('--diagnostics-dir') + 1]
    assert diagnostics.endswith('out/lidar_camera_alignment_diagnostics')
    assert alignment[alignment.index('--worst-views') + 1] == '6'


def test_depth_support_options_reach_calibration_and_alignment(tmp_path):
    args = _args(
        tmp_path, '--refine-spatiotemporal-calibration',
        '--calibration-depth-support-radius', '2',
        '--calibration-depth-support-min-neighbors', '5',
        '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--alignment-depth-support-radius', '3',
        '--alignment-depth-support-min-neighbors', '7')
    commands = dict(cmp.build_commands(args))
    calibration = commands['spatiotemporal calibration']
    alignment = commands['camera-LiDAR alignment']
    assert calibration[calibration.index('--depth-support-radius') + 1] == '2'
    assert calibration[calibration.index('--depth-support-min-neighbors') + 1] == '5'
    assert calibration[
        calibration.index('--minimum-supported-edge-fraction') + 1] == '0.25'
    assert alignment[alignment.index('--depth-support-radius') + 1] == '3'
    assert alignment[alignment.index('--depth-support-min-neighbors') + 1] == '7'


def test_fixed_contour_options_reach_calibration_and_alignment(tmp_path):
    args = _args(
        tmp_path, '--refine-spatiotemporal-calibration',
        '--calibration-fixed-contours',
        '--calibration-contour-max-points-per-view', '12000',
        '--calibration-orientation-max-angle-deg', '25',
        '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--alignment-fixed-contours',
        '--alignment-contour-association-distance', '30')
    commands = dict(cmp.build_commands(args))
    calibration = commands['spatiotemporal calibration']
    alignment = commands['camera-LiDAR alignment']
    assert '--fixed-contours' in calibration
    assert calibration[
        calibration.index('--contour-max-points-per-view') + 1] == '12000'
    assert calibration[
        calibration.index('--orientation-max-angle-deg') + 1] == '25.0'
    assert '--fixed-contours' in alignment
    assert alignment[
        alignment.index('--contour-association-distance') + 1] == '30'


def test_vignette_and_confidence_options_reach_map_builder(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--color-image-margin', '140', '--color-min-samples', '3',
        '--color-vignette-gain-limit', '2.5', '--color-overlap-balance',
        '--color-view-confidence', '--color-normal-voxel', '0.2',
        '--color-min-view-cosine', '0.1', '--color-min-projected-scale', '8',
        '--color-view-score-power', '2'))
    build = commands[1][1]
    assert build[build.index('--color-image-margin') + 1] == '140'
    assert build[build.index('--color-min-samples') + 1] == '3'
    assert build[build.index('--color-vignette-gain-limit') + 1] == '2.5'
    assert '--color-overlap-balance' in build
    assert '--color-view-confidence' in build
    assert build[build.index('--color-normal-voxel') + 1] == '0.2'
    assert build[build.index('--color-min-view-cosine') + 1] == '0.1'
    assert build[build.index('--color-min-projected-scale') + 1] == '8.0'
    assert build[build.index('--color-view-score-power') + 1] == '2.0'


def test_vignette_and_confidence_defaults_are_off(tmp_path):
    build = cmp.build_commands(_args(tmp_path))[1][1]
    assert build[build.index('--color-image-margin') + 1] == '0'
    assert build[build.index('--color-min-samples') + 1] == '1'
    assert build[build.index('--color-vignette-gain-limit') + 1] == '1.0'
    assert '--color-overlap-balance' not in build
    assert '--color-view-confidence' not in build


def test_dynamic_map_cleaner_reaches_builder_with_provenance(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--dynamic-map-cleaner', 'fusion',
        '--dynamic-map-cleaner-workers', '4',
        '--dynamic-map-cleaner-evidence-stride', '5',
        '--dynamic-map-cleaner-free-votes-fraction', '0.7',
        '--dynamic-map-cleaner-free-votes-floor', '3',
        '--dynamic-map-cleaner-void-min-scans', '4'))
    build = dict(commands)['coloured map']
    assert build[build.index('--dynamic-map-cleaner') + 1] == 'fusion'
    assert build[build.index('--dynamic-map-cleaner-workers') + 1] == '4'
    assert build[
        build.index('--dynamic-map-cleaner-evidence-stride') + 1] == '5'
    assert build[
        build.index('--dynamic-map-cleaner-free-votes-fraction') + 1] == '0.7'
    assert build[
        build.index('--dynamic-map-cleaner-free-votes-floor') + 1] == '3'
    assert build[
        build.index('--dynamic-map-cleaner-void-min-scans') + 1] == '4'
    assert build[build.index('--dynamic-map-cleaner-report') + 1].endswith(
        'out/dynamic_map_cleaning.json')


def test_dynamic_map_cleaner_is_default_off(tmp_path):
    build = dict(cmp.build_commands(_args(tmp_path)))['coloured map']
    assert '--dynamic-map-cleaner' not in build


def test_quality_profile_adds_appearance_stage_and_gate_wiring(tmp_path):
    for name in ('profile.yaml', 'traj_report.json', 'geom_report.yaml'):
        (tmp_path / name).write_text('{}')
    commands = cmp.build_commands(_args(
        tmp_path, '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--trajectory-report', str(tmp_path / 'traj_report.json'),
        '--geometry-report', str(tmp_path / 'geom_report.yaml')))
    names = [name for name, _ in commands]
    assert 'appearance' in names
    appearance = dict(commands)['appearance']
    assert appearance[appearance.index('--out') + 1].endswith(
        'colored_map_appearance.json')
    gate_cmd = dict(commands)['quality gate']
    assert gate_cmd[gate_cmd.index('--appearance-report') + 1].endswith(
        'colored_map_appearance.json')


def test_planar_roughness_option_reaches_appearance_evaluator(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--quality-profile', str(tmp_path / 'profile.yaml'),
        '--appearance-planar-roughness'))
    assert '--planar-roughness' in dict(commands)['appearance']


def test_planar_roughness_profile_enables_evaluator_automatically(tmp_path):
    profile = tmp_path / 'profile.yaml'
    profile.write_text(
        'colored_map_quality_profile:\n'
        '  thresholds:\n'
        '    appearance_planar_roughness_p90_max: 25.0\n')
    commands = cmp.build_commands(_args(
        tmp_path, '--quality-profile', str(profile)))
    assert '--planar-roughness' in dict(commands)['appearance']
