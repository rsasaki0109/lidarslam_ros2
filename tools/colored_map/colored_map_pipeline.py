#!/usr/bin/env python3
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

"""Build a robust coloured SLAM map from one bag and a TUM trajectory.

This is the user-facing entry point over ``extract_posed_images.py`` and
``build_lidar_init.py``. Existing posed images and maps are reused by default,
so an interrupted or repeated run only performs missing work.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Sequence

import numpy as np
import yaml


TOOL_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOL_DIR.parents[1]


def effective_trajectory(args) -> Path:
    """Return the trajectory consumed by image and map generation."""
    if args.raw_traj is None:
        return Path(args.traj)
    return Path(args.out) / 'dense_corrected_trajectory.tum'


def is_stale(output: Path, inputs: Sequence[Path]) -> bool:
    """Return whether an existing output predates any available input."""
    if not output.is_file():
        return True
    output_mtime = output.stat().st_mtime_ns
    return any(path.exists() and path.stat().st_mtime_ns > output_mtime
               for path in inputs)


def profile_uses_planar_roughness(path: Path) -> bool:
    """Return whether a readable quality profile gates planar roughness."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        thresholds = document['colored_map_quality_profile']['thresholds']
    except (OSError, TypeError, KeyError, yaml.YAMLError):
        return False
    return any(str(key).startswith('appearance_planar_roughness_')
               for key in thresholds)


def report_has_metric(path: Path, metric: str) -> bool:
    """Return whether an existing JSON report contains a top-level metric."""
    try:
        return metric in json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError, TypeError):
        return False


def validate_trajectory_density(path: Path, max_gap: float) -> None:
    """Reject sparse graph keyframes that cannot register individual scans."""
    if max_gap <= 0:
        return
    stamps = []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and not fields[0].startswith('#'):
            stamps.append(float(fields[0]))
    if len(stamps) < 2:
        raise ValueError(f'{path}: trajectory needs at least two poses')
    largest = max(b - a for a, b in zip(stamps, stamps[1:]))
    if largest > max_gap:
        raise ValueError(
            f'{path}: trajectory pose gap {largest:.3f}s exceeds '
            f'--max-trajectory-gap {max_gap:.3f}s; use a dense SLAM '
            'trajectory rather than sparse pose-graph keyframes')


def validate_colour_source(transforms: Path, allow_monochrome: bool = False
                           ) -> dict:
    """Reject mono posed images unless the caller explicitly allows them."""
    import imageio as iio

    document = json.loads(Path(transforms).read_text())
    frames = document.get('frames', [])
    if not frames:
        raise ValueError(f'{transforms}: no posed image frames')
    sampled = frames[::max(1, len(frames) // 5)][:5]
    deltas = []
    channels = []
    for frame in sampled:
        path = (Path(transforms).parent / frame['file_path']).resolve()
        image = np.asarray(iio.imread(path))
        channels.append(1 if image.ndim == 2 else image.shape[2])
        if image.ndim == 3 and image.shape[2] >= 3:
            deltas.append(float(np.mean(np.ptp(image[:, :, :3], axis=2))))
        else:
            deltas.append(0.0)
    colour_delta = float(np.mean(deltas))
    is_colour = max(channels) >= 3 and colour_delta > 0.5
    report = {'sampled_frames': len(sampled), 'max_channels': max(channels),
              'mean_channel_delta': colour_delta, 'is_colour': is_colour}
    if not is_colour and not allow_monochrome:
        raise ValueError(
            'camera source is monochrome; RGB point colouring would only copy '
            'luminance into three channels. Choose a bgr8/rgb8 camera topic, or '
            'pass --allow-monochrome for geometry-only benchmarking')
    return report


def build_commands(args) -> list[tuple[str, list[str]]]:
    """Return the missing/forced pipeline stages as ``(name, argv)`` pairs."""
    out_dir = Path(args.out)
    posed_dir = out_dir / 'posed_images'
    extracted_transforms = posed_dir / 'transforms.json'
    masked_transforms = posed_dir / 'transforms_dynamic_masks.json'
    refined_transforms = posed_dir / 'transforms_spatiotemporal.json'
    calibration_transforms = (
        masked_transforms if args.dynamic_mask_dir is not None
        else extracted_transforms)
    transforms = (refined_transforms if args.refine_spatiotemporal_calibration
                  else calibration_transforms)
    colored_map = out_dir / 'colored_map.ply'
    extrinsic_path = (Path(args.extrinsic) if args.extrinsic is not None else
                      out_dir / 'generated_body_camera_extrinsic.json')
    commands = []
    trajectory = effective_trajectory(args)

    rebuild_trajectory = (args.raw_traj is not None and (
        args.force_trajectory or is_stale(
            trajectory, [Path(args.raw_traj), Path(args.traj)])))
    if rebuild_trajectory:
        commands.append(('dense corrected trajectory', [
            sys.executable,
            str(REPO_ROOT / 'scripts' / 'densify_corrected_trajectory.py'),
            '--raw', str(args.raw_traj), '--corrected', str(args.traj),
            '--output', str(trajectory),
            '--max-anchor-offset', str(args.max_anchor_offset),
        ]))

    rebuild_images = (rebuild_trajectory or args.force_images or
                      is_stale(extracted_transforms, [trajectory]))
    if rebuild_images:
        extract = [
            sys.executable, str(TOOL_DIR / 'extract_posed_images.py'),
            '--bag', str(args.bag), '--traj', str(trajectory),
            '--camera-topic', args.camera_topic,
            '--camera-info-topic', args.camera_info_topic,
            '--extrinsic', str(extrinsic_path), '--out', str(posed_dir),
            '--time-offset', args.time_offset,
            '--time-offset-adjustment', str(args.time_offset_adjustment),
            '--clock-reference-topic', args.points_topic,
            '--stride', str(args.image_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
        ]
        if not args.no_undistort:
            extract.append('--undistort')
        if args.intrinsics_yaml is not None:
            extract.extend(['--intrinsics-yaml', str(args.intrinsics_yaml)])
        commands.append(('posed images', extract))

    rebuild_masks = False
    if args.dynamic_mask_dir is not None:
        mask_dir = Path(args.dynamic_mask_dir)
        mask_inputs = [extracted_transforms, mask_dir]
        if mask_dir.is_dir():
            mask_inputs.extend(sorted(mask_dir.glob('*.png')))
        rebuild_masks = (
            rebuild_images or args.force_dynamic_masks or
            is_stale(masked_transforms, mask_inputs))
        if rebuild_masks:
            attach = [
                sys.executable, str(TOOL_DIR / 'attach_dynamic_image_masks.py'),
                '--transforms', str(extracted_transforms),
                '--mask-dir', str(mask_dir), '--out', str(masked_transforms),
            ]
            if args.allow_missing_dynamic_masks:
                attach.append('--allow-missing')
            commands.append(('dynamic image masks', attach))

    def map_command(output: Path, color_transforms: Path | None = None) -> list[str]:
        command = [
            sys.executable, str(TOOL_DIR / 'build_lidar_init.py'),
            '--bag', str(args.bag), '--traj', str(trajectory),
            '--points-topic', args.points_topic, '--out', str(output),
            '--voxel', str(args.voxel), '--max-points', str(args.max_points),
            '--min-range', str(args.min_range), '--max-range', str(args.max_range),
            '--min-neighbors', str(args.min_neighbors),
            '--sparse-voxel', str(args.sparse_voxel),
            '--stride', str(args.scan_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
        ]
        if args.dynamic_map_cleaner != 'none':
            command.extend([
                '--dynamic-map-cleaner', args.dynamic_map_cleaner,
                '--dynamic-map-cleaner-workers',
                str(args.dynamic_map_cleaner_workers),
                '--dynamic-map-cleaner-evidence-stride',
                str(args.dynamic_map_cleaner_evidence_stride),
                '--dynamic-map-cleaner-free-votes-fraction',
                str(args.dynamic_map_cleaner_free_votes_fraction),
                '--dynamic-map-cleaner-free-votes-floor',
                str(args.dynamic_map_cleaner_free_votes_floor),
                '--dynamic-map-cleaner-void-min-scans',
                str(args.dynamic_map_cleaner_void_min_scans),
                '--dynamic-map-cleaner-report',
                str(out_dir / 'dynamic_map_cleaning.json'),
            ])
        if args.lidar_calibration is not None:
            command.extend([
                '--lidar-calibration', str(args.lidar_calibration),
                '--lidar-key', args.lidar_key,
            ])
        if args.no_deskew:
            command.append('--no-deskew')
        if color_transforms is not None:
            command.extend([
                '--color-transforms', str(color_transforms), '--color-robust',
                '--color-exposure-scale-limit',
                str(args.color_exposure_scale_limit),
                '--color-max-samples', str(args.color_max_samples),
                '--color-image-margin', str(args.color_image_margin),
                '--color-vignette-gain-limit',
                str(args.color_vignette_gain_limit),
                '--color-min-samples', str(args.color_min_samples),
            ])
            if not args.color_normalize_exposure:
                command.append('--color-no-normalize-exposure')
            if args.color_overlap_balance:
                command.append('--color-overlap-balance')
            if args.color_view_confidence:
                command.extend([
                    '--color-view-confidence',
                    '--color-normal-voxel', str(args.color_normal_voxel),
                    '--color-min-view-cosine', str(args.color_min_view_cosine),
                    '--color-min-projected-scale',
                    str(args.color_min_projected_scale),
                    '--color-view-score-power', str(args.color_view_score_power),
                ])
            if args.color_geometry_aware:
                command.extend([
                    '--color-geometry-aware',
                    '--color-occlusion-margin-px',
                    str(args.color_occlusion_margin_px),
                    '--color-depth-edge-margin-px',
                    str(args.color_depth_edge_margin_px),
                    '--color-depth-edge-tolerance',
                    str(args.color_depth_edge_tolerance),
                    '--color-depth-edge-relative-tolerance',
                    str(args.color_depth_edge_relative_tolerance),
                    '--color-dynamic-mask-margin-px',
                    str(args.color_dynamic_mask_margin_px),
                    '--color-calibration-sigma-multiplier',
                    str(args.color_calibration_sigma_multiplier),
                    '--color-max-uncertainty-margin-px',
                    str(args.color_max_uncertainty_margin_px),
                ])
                if args.color_dynamic_exclusion:
                    command.append('--color-dynamic-exclusion')
        return command

    rebuild_calibration = False
    if args.refine_spatiotemporal_calibration:
        calibration_cloud = out_dir / 'spatiotemporal_calibration_geometry.ply'
        calibration_report = out_dir / 'spatiotemporal_calibration.json'
        rebuild_calibration_cloud = (
            rebuild_trajectory or args.force_map or args.force_calibration or
            is_stale(calibration_cloud, [trajectory]))
        if rebuild_calibration_cloud:
            commands.append(('calibration geometry', map_command(
                calibration_cloud)))
        rebuild_calibration = (
            rebuild_masks or rebuild_calibration_cloud or
            args.force_calibration or
            is_stale(refined_transforms, [trajectory, calibration_transforms,
                                          calibration_cloud]) or
            is_stale(calibration_report, [trajectory, calibration_transforms,
                                          calibration_cloud]))
        if rebuild_calibration:
            commands.append(('spatiotemporal calibration', [
                sys.executable,
                str(REPO_ROOT / 'scripts' /
                    'evaluate_lidar_camera_alignment.py'),
                '--pointcloud', str(calibration_cloud),
                '--transforms', str(calibration_transforms),
                '--trajectory', str(trajectory),
                '--out', str(calibration_report),
                '--optimize-spatiotemporal',
                '--production-calibration',
                '--corrected-transforms-out', str(refined_transforms),
                '--view-stride', str(args.calibration_view_stride),
                '--max-points', str(args.calibration_max_points),
                '--optimization-rounds', str(args.calibration_rounds),
                '--time-step', str(args.calibration_time_step),
                '--translation-step', str(args.calibration_translation_step),
                '--rotation-step-deg', str(args.calibration_rotation_step_deg),
                '--max-time-offset', str(args.calibration_max_time_offset),
                '--max-translation', str(args.calibration_max_translation),
                '--max-rotation-deg', str(args.calibration_max_rotation_deg),
                '--holdout-modulo', str(args.calibration_holdout_modulo),
                '--holdout-fraction', str(args.calibration_holdout_fraction),
                '--spatial-segments', str(args.calibration_spatial_segments),
                '--pyramid-scales', args.calibration_pyramid_scales,
                '--rounds-per-pyramid-level',
                str(args.calibration_rounds_per_pyramid_level),
                '--auto-bound-expansions',
                str(args.calibration_auto_bound_expansions),
                '--bound-expansion-factor',
                str(args.calibration_bound_expansion_factor),
                '--observability-scale',
                str(args.calibration_observability_scale),
                '--minimum-curvature',
                str(args.calibration_minimum_curvature),
                '--maximum-condition',
                str(args.calibration_maximum_condition),
                '--maximum-time-translation-correlation',
                str(args.calibration_maximum_time_translation_correlation),
                '--minimum-edge-points',
                str(args.calibration_minimum_edge_points),
                '--minimum-heldout-improvement',
                str(args.calibration_minimum_heldout_improvement),
            ] + ([
                '--depth-support-radius',
                str(args.calibration_depth_support_radius),
                '--depth-support-min-neighbors',
                str(args.calibration_depth_support_min_neighbors),
                '--depth-support-absolute',
                str(args.calibration_depth_support_absolute),
                '--depth-support-relative',
                str(args.calibration_depth_support_relative),
                '--minimum-supported-edge-fraction',
                str(args.calibration_minimum_supported_edge_fraction),
            ] if args.calibration_depth_support_radius > 0 else []) + ([
                '--fixed-contours',
                '--contour-association-distance',
                str(args.calibration_contour_association_distance),
                '--contour-max-points-per-view',
                str(args.calibration_contour_max_points_per_view),
                '--contour-min-points-per-view',
                str(args.calibration_contour_min_points_per_view),
                '--orientation-max-angle-deg',
                str(args.calibration_orientation_max_angle_deg),
            ] if args.calibration_fixed_contours else [])))

    if (rebuild_images or rebuild_masks or rebuild_calibration or
            args.force_map or
            is_stale(colored_map, [trajectory, transforms])):
        commands.append(('coloured map', map_command(colored_map, transforms)))

    if args.quality_profile is not None:
        alignment_report = out_dir / 'lidar_camera_alignment.json'
        alignment_diagnostics = out_dir / 'lidar_camera_alignment_diagnostics'
        colour_report = out_dir / 'heldout_point_colors.json'
        appearance_report = out_dir / 'colored_map_appearance.json'
        quality_report = out_dir / 'colored_map_quality_gate.json'
        rebuild_map = any(name == 'coloured map' for name, _ in commands)
        rebuild_images = any(name == 'posed images' for name, _ in commands)
        planar_roughness = (
            args.appearance_planar_roughness or
            profile_uses_planar_roughness(Path(args.quality_profile)))
        if (rebuild_map or rebuild_images or args.force_quality or
                (args.alignment_diagnostics and not
                 (alignment_diagnostics / 'diagnostics.json').is_file()) or
                is_stale(alignment_report, [colored_map, transforms])):
            alignment_command = [
                sys.executable,
                str(REPO_ROOT / 'scripts' /
                    'evaluate_lidar_camera_alignment.py'),
                '--pointcloud', str(colored_map),
                '--transforms', str(transforms),
                '--out', str(alignment_report),
            ]
            if args.alignment_diagnostics:
                alignment_command.extend([
                    '--diagnostics-dir', str(alignment_diagnostics),
                    '--worst-views', str(args.alignment_diagnostic_worst_views),
                ])
            if args.alignment_depth_support_radius > 0:
                alignment_command.extend([
                    '--depth-support-radius',
                    str(args.alignment_depth_support_radius),
                    '--depth-support-min-neighbors',
                    str(args.alignment_depth_support_min_neighbors),
                    '--depth-support-absolute',
                    str(args.alignment_depth_support_absolute),
                    '--depth-support-relative',
                    str(args.alignment_depth_support_relative),
                ])
            if args.alignment_fixed_contours:
                alignment_command.extend([
                    '--fixed-contours',
                    '--contour-association-distance',
                    str(args.alignment_contour_association_distance),
                    '--contour-max-points-per-view',
                    str(args.alignment_contour_max_points_per_view),
                    '--contour-min-points-per-view',
                    str(args.alignment_contour_min_points_per_view),
                    '--orientation-max-angle-deg',
                    str(args.alignment_orientation_max_angle_deg),
                ])
            commands.append(('camera-LiDAR alignment', alignment_command))
        if (rebuild_map or rebuild_images or args.force_quality or
                is_stale(colour_report, [colored_map, transforms])):
            commands.append(('held-out colour', [
                sys.executable,
                str(REPO_ROOT / 'scripts' / 'evaluate_heldout_point_colors.py'),
                '--pointcloud', str(colored_map),
                '--transforms', str(transforms),
                '--out', str(colour_report),
            ]))
        if (rebuild_map or rebuild_images or args.force_quality or
                is_stale(appearance_report, [colored_map, transforms]) or
                (planar_roughness and not report_has_metric(
                    appearance_report, 'planar_roughness'))):
            appearance_command = [
                sys.executable,
                str(REPO_ROOT / 'scripts' /
                    'evaluate_colored_map_appearance.py'),
                '--pointcloud', str(colored_map),
                '--transforms', str(transforms),
                '--out', str(appearance_report),
            ]
            if planar_roughness:
                appearance_command.append('--planar-roughness')
            commands.append(('appearance', appearance_command))
        gate_inputs = [alignment_report, colour_report, appearance_report,
                       Path(args.quality_profile)]
        gate_inputs.extend(Path(path) for path in
                           (args.trajectory_report, args.geometry_report)
                           if path is not None)
        if (args.force_quality or
                any(name in ('camera-LiDAR alignment', 'held-out colour',
                             'appearance')
                    for name, _ in commands) or
                is_stale(quality_report, gate_inputs)):
            gate_command = [
                sys.executable,
                str(REPO_ROOT / 'scripts' / 'check_colored_map_quality.py'),
                '--alignment-report', str(alignment_report),
                '--colour-report', str(colour_report),
                '--appearance-report', str(appearance_report),
                '--profile', str(args.quality_profile),
                '--out', str(quality_report),
            ]
            for option, path in (
                    ('--trajectory-report', args.trajectory_report),
                    ('--geometry-report', args.geometry_report)):
                if path is not None:
                    gate_command.extend((option, str(path)))
            commands.append(('quality gate', gate_command))
    return commands


def run_pipeline(args) -> dict:
    """Execute missing stages and return paths plus the stages that ran."""
    out_dir = Path(args.out)
    try:
        calibration_scales = tuple(
            float(item) for item in args.calibration_pyramid_scales.split(','))
    except ValueError as exc:
        raise ValueError('calibration pyramid scales must be numbers') from exc
    if args.kalibr_camchain is not None and args.lidar_calibration is None:
        raise ValueError('--kalibr-camchain requires --lidar-calibration')
    if args.force_calibration and not args.refine_spatiotemporal_calibration:
        raise ValueError(
            '--force-calibration requires --refine-spatiotemporal-calibration')
    if args.color_dynamic_exclusion and not args.color_geometry_aware:
        raise ValueError('--color-dynamic-exclusion requires '
                         '--color-geometry-aware')
    if args.color_dynamic_exclusion and args.dynamic_mask_dir is None:
        raise ValueError('--color-dynamic-exclusion requires '
                         '--dynamic-mask-dir')
    if args.force_dynamic_masks and args.dynamic_mask_dir is None:
        raise ValueError('--force-dynamic-masks requires --dynamic-mask-dir')
    if (args.color_dynamic_exclusion and
            args.allow_missing_dynamic_masks):
        raise ValueError('dynamic exclusion requires complete masks; remove '
                         '--allow-missing-dynamic-masks')
    if (args.color_calibration_sigma_multiplier > 0.0 and
            not args.refine_spatiotemporal_calibration):
        raise ValueError('calibration uncertainty fusion requires '
                         '--refine-spatiotemporal-calibration')
    if (args.color_occlusion_margin_px < 0 or
            args.color_depth_edge_margin_px < 0 or
            args.color_depth_edge_tolerance < 0.0 or
            args.color_depth_edge_relative_tolerance < 0.0 or
            args.color_dynamic_mask_margin_px < 0 or
            args.color_calibration_sigma_multiplier < 0.0 or
            args.color_max_uncertainty_margin_px < 0):
        raise ValueError('invalid geometry-aware colour fusion settings')
    if (args.dynamic_map_cleaner_workers < 1 or
            args.dynamic_map_cleaner_evidence_stride < 1 or
            not 0.0 < args.dynamic_map_cleaner_free_votes_fraction <= 1.0 or
            args.dynamic_map_cleaner_free_votes_floor < 1 or
            args.dynamic_map_cleaner_void_min_scans < 1):
        raise ValueError('invalid dynamic map cleaner settings')
    if args.alignment_diagnostic_worst_views < 1:
        raise ValueError('--alignment-diagnostic-worst-views must be >= 1')
    for prefix, radius, neighbours, absolute, relative in (
            ('calibration', args.calibration_depth_support_radius,
             args.calibration_depth_support_min_neighbors,
             args.calibration_depth_support_absolute,
             args.calibration_depth_support_relative),
            ('alignment', args.alignment_depth_support_radius,
             args.alignment_depth_support_min_neighbors,
             args.alignment_depth_support_absolute,
             args.alignment_depth_support_relative)):
        if (radius < 0 or neighbours < 1 or absolute < 0.0 or relative < 0.0 or
                (radius > 0 and neighbours > (2 * radius + 1) ** 2 - 1)):
            raise ValueError(f'invalid {prefix} depth support settings')
    if not 0.0 <= args.calibration_minimum_supported_edge_fraction <= 1.0:
        raise ValueError('invalid calibration minimum supported edge fraction')
    for prefix, association, maximum, minimum in (
            ('calibration', args.calibration_contour_association_distance,
             args.calibration_contour_max_points_per_view,
             args.calibration_contour_min_points_per_view),
            ('alignment', args.alignment_contour_association_distance,
             args.alignment_contour_max_points_per_view,
             args.alignment_contour_min_points_per_view)):
        if ((association != 0 and association < 12) or maximum < 1 or
                minimum < 1 or minimum > maximum):
            raise ValueError(f'invalid {prefix} fixed contour settings')
    if (not 0.0 <= args.calibration_orientation_max_angle_deg <= 90.0 or
            not 0.0 <= args.alignment_orientation_max_angle_deg <= 90.0):
        raise ValueError('invalid contour orientation angle')
    if (args.calibration_orientation_max_angle_deg > 0.0 and
            not args.calibration_fixed_contours):
        raise ValueError('calibration orientation gate requires fixed contours')
    if (args.alignment_orientation_max_angle_deg > 0.0 and
            not args.alignment_fixed_contours):
        raise ValueError('alignment orientation gate requires fixed contours')
    if args.refine_spatiotemporal_calibration and (
            args.calibration_view_stride < 1 or
            args.calibration_max_points < 1 or
            args.calibration_rounds < 1 or
            args.calibration_time_step <= 0.0 or
            args.calibration_translation_step <= 0.0 or
            args.calibration_rotation_step_deg <= 0.0 or
            args.calibration_max_time_offset < 0.0 or
            args.calibration_max_translation < 0.0 or
            args.calibration_max_rotation_deg < 0.0 or
            args.calibration_holdout_modulo < 2 or
            not 0.0 < args.calibration_holdout_fraction < 0.5 or
            args.calibration_spatial_segments < 1 or
            args.calibration_rounds_per_pyramid_level < 1 or
            args.calibration_auto_bound_expansions < 0 or
            args.calibration_bound_expansion_factor <= 1.0 or
            not 0.0 < args.calibration_observability_scale <= 1.0 or
            args.calibration_minimum_curvature <= 0.0 or
            args.calibration_maximum_condition <= 1.0 or
            not 0.0 <= args.calibration_maximum_time_translation_correlation < 1.0 or
            args.calibration_minimum_edge_points < 1 or
            not 0.0 <= args.calibration_minimum_heldout_improvement < 1.0):
        raise ValueError('invalid spatiotemporal calibration search settings')
    if (args.refine_spatiotemporal_calibration and
            (not calibration_scales or
             tuple(sorted(calibration_scales)) != calibration_scales or
             any(not 0.0 < scale <= 1.0 for scale in calibration_scales) or
             not np.isclose(args.calibration_observability_scale,
                            calibration_scales[-1]))):
        raise ValueError('calibration observability scale must match the '
                         'finest valid pyramid scale')
    if not args.dry_run:
        out_dir.mkdir(parents=True, exist_ok=True)
        if args.kalibr_camchain is not None:
            from extract_posed_images import load_kalibr_body_camera_extrinsic
            matrix = load_kalibr_body_camera_extrinsic(
                args.kalibr_camchain, args.lidar_calibration,
                camera_key=args.camera_key, lidar_key=args.lidar_key)
            generated = out_dir / 'generated_body_camera_extrinsic.json'
            generated.write_text(json.dumps({'matrix': matrix.tolist()}, indent=2))
    commands = build_commands(args)
    trajectory = effective_trajectory(args)
    trajectory_validated = False
    for name, command in commands:
        if (not args.dry_run and name != 'dense corrected trajectory' and
                not trajectory_validated):
            validate_trajectory_density(
                trajectory, args.max_trajectory_gap)
            trajectory_validated = True
        if not args.dry_run and name == 'coloured map':
            validate_colour_source(
                out_dir / 'posed_images' / 'transforms.json',
                args.allow_monochrome)
        print(f'[{name}]', ' '.join(command))
        if not args.dry_run:
            subprocess.run(command, check=True)
    if not args.dry_run and not trajectory_validated:
        validate_trajectory_density(trajectory, args.max_trajectory_gap)
    return {
        'stages': [name for name, _ in commands],
        'transforms': out_dir / 'posed_images' / (
            'transforms_spatiotemporal.json'
            if args.refine_spatiotemporal_calibration else
            ('transforms_dynamic_masks.json'
             if args.dynamic_mask_dir is not None else 'transforms.json')),
        'colored_map': out_dir / 'colored_map.ply',
        'trajectory': trajectory,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('bag', type=Path, help='rosbag2 directory')
    p.add_argument('traj', type=Path,
                   help='corrected SLAM trajectory (TUM, world<-body)')
    p.add_argument('out', type=Path, help='output directory')
    p.add_argument('--raw-traj', type=Path,
                   help='dense pre-optimization TUM trajectory; propagate the '
                        'corrections from traj before colouring')
    p.add_argument('--max-anchor-offset', type=float, default=0.2,
                   help='allowed corrected-anchor extrapolation into the raw '
                        'trajectory (s)')
    calibration = p.add_mutually_exclusive_group(required=True)
    calibration.add_argument('--extrinsic', type=Path,
                             help='body<-camera YAML/JSON or vlcal calib.json')
    calibration.add_argument('--kalibr-camchain', type=Path,
                             help='Kalibr camera chain with T_cam_imu')
    p.add_argument('--lidar-calibration', type=Path,
                   help='parented LiDAR calibration paired with Kalibr camchain')
    p.add_argument('--camera-key', default='cam0')
    p.add_argument('--lidar-key', default='PandarXT-32')
    p.add_argument('--points-topic', default='/livox/points')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--intrinsics-yaml', type=Path,
                   help='camera intrinsics YAML when the bag has no CameraInfo')
    p.add_argument('--time-offset', default='auto',
                   help='camera-to-trajectory clock offset or auto')
    p.add_argument('--time-offset-adjustment', type=float, default=0.0,
                   help='seconds added after fixed or auto clock correction')
    p.add_argument('--refine-spatiotemporal-calibration', action='store_true',
                   help='optimize camera time offset and 6DoF extrinsic on '
                        'LiDAR/image edges, accepting only held-out improvement')
    p.add_argument('--calibration-view-stride', type=int, default=10)
    p.add_argument('--calibration-max-points', type=int, default=300000)
    p.add_argument('--calibration-rounds', type=int, default=3)
    p.add_argument('--calibration-time-step', type=float, default=0.02)
    p.add_argument('--calibration-translation-step', type=float, default=0.02)
    p.add_argument('--calibration-rotation-step-deg', type=float, default=0.2)
    p.add_argument('--calibration-max-time-offset', type=float, default=0.1)
    p.add_argument('--calibration-max-translation', type=float, default=0.1)
    p.add_argument('--calibration-max-rotation-deg', type=float, default=2.0)
    p.add_argument('--calibration-holdout-modulo', type=int, default=5)
    p.add_argument('--calibration-holdout-fraction', type=float, default=0.2)
    p.add_argument('--calibration-spatial-segments', type=int, default=4)
    p.add_argument('--calibration-pyramid-scales', default='0.25,0.5,1.0')
    p.add_argument('--calibration-rounds-per-pyramid-level', type=int,
                   default=2)
    p.add_argument('--calibration-auto-bound-expansions', type=int, default=2)
    p.add_argument('--calibration-bound-expansion-factor', type=float,
                   default=2.0)
    p.add_argument('--calibration-observability-scale', type=float, default=1.0)
    p.add_argument('--calibration-minimum-curvature', type=float, default=1e-6)
    p.add_argument('--calibration-maximum-condition', type=float, default=1e6)
    p.add_argument('--calibration-maximum-time-translation-correlation',
                   type=float, default=0.98)
    p.add_argument('--calibration-minimum-edge-points', type=int, default=50)
    p.add_argument('--calibration-minimum-heldout-improvement', type=float,
                   default=0.0)
    p.add_argument('--calibration-depth-support-radius', type=int, default=0)
    p.add_argument('--calibration-depth-support-min-neighbors', type=int,
                   default=4)
    p.add_argument('--calibration-depth-support-absolute', type=float,
                   default=0.10)
    p.add_argument('--calibration-depth-support-relative', type=float,
                   default=0.01)
    p.add_argument('--calibration-minimum-supported-edge-fraction', type=float,
                   default=0.25)
    p.add_argument('--calibration-fixed-contours', action='store_true')
    p.add_argument('--calibration-contour-association-distance', type=int,
                   default=0)
    p.add_argument('--calibration-contour-max-points-per-view', type=int,
                   default=50000)
    p.add_argument('--calibration-contour-min-points-per-view', type=int,
                   default=500)
    p.add_argument('--calibration-orientation-max-angle-deg', type=float,
                   default=0.0)
    p.add_argument('--max-trajectory-gap', type=float, default=0.5,
                   help='reject sparse pose streams with a larger gap (s); '
                        'set <=0 to disable')
    p.add_argument('--image-stride', type=int, default=1)
    p.add_argument('--scan-stride', type=int, default=1)
    p.add_argument('--no-deskew', action='store_true',
                   help='accumulate each scan at its scan pose; use when the '
                        'trajectory is registration-at-scan-rate rather than '
                        'a continuous-time body trajectory')
    p.add_argument('--voxel', type=float, default=0.1)
    p.add_argument('--max-points', type=int, default=300000)
    p.add_argument('--dynamic-map-cleaner',
                   choices=('none', 'fusion', 'range', 'scan_ratio'),
                   default='none')
    p.add_argument('--dynamic-map-cleaner-workers', type=int, default=1)
    p.add_argument('--dynamic-map-cleaner-evidence-stride', type=int,
                   default=1)
    p.add_argument('--dynamic-map-cleaner-free-votes-fraction', type=float,
                   default=0.9)
    p.add_argument('--dynamic-map-cleaner-free-votes-floor', type=int,
                   default=2)
    p.add_argument('--dynamic-map-cleaner-void-min-scans', type=int,
                   default=11)
    p.add_argument('--min-range', type=float, default=0.0)
    p.add_argument('--max-range', type=float, default=80.0)
    p.add_argument('--min-neighbors', type=int, default=2,
                   help='minimum points in the local density neighbourhood; '
                        'default 2 removes isolated LiDAR returns, 0 disables')
    p.add_argument('--sparse-voxel', type=float, default=0.1,
                   help='density-neighbourhood voxel size (m)')
    p.add_argument('--start-time', type=float, default=0.0)
    p.add_argument('--end-time', type=float, default=-1.0)
    p.add_argument('--no-undistort', action='store_true')
    p.add_argument('--allow-monochrome', action='store_true',
                   help='allow luminance-only maps for geometry benchmarks')
    p.add_argument('--color-no-normalize-exposure', action='store_false',
                   dest='color_normalize_exposure',
                   help='ablate per-view exposure normalization while retaining '
                        'occlusion-aware RGB medoid fusion')
    p.add_argument('--color-exposure-scale-limit', type=float, default=1.5)
    p.add_argument('--color-max-samples', type=int, default=12)
    p.add_argument('--color-image-margin', type=int, default=0,
                   help='ignore colour samples within this many pixels of the '
                        'image border (lens vignette)')
    p.add_argument('--color-vignette-gain-limit', type=float, default=1.0,
                   help='maximum automatic radial luminance gain; 1 disables')
    p.add_argument('--color-overlap-balance', action='store_true')
    p.add_argument('--color-view-confidence', action='store_true')
    p.add_argument('--color-normal-voxel', type=float, default=0.12)
    p.add_argument('--color-min-view-cosine', type=float, default=0.0)
    p.add_argument('--color-min-projected-scale', type=float, default=0.0)
    p.add_argument('--color-view-score-power', type=float, default=1.0)
    p.add_argument('--color-geometry-aware', action='store_true')
    p.add_argument('--color-occlusion-margin-px', type=int, default=0)
    p.add_argument('--color-depth-edge-margin-px', type=int, default=0)
    p.add_argument('--color-depth-edge-tolerance', type=float, default=1.0)
    p.add_argument('--color-depth-edge-relative-tolerance', type=float,
                   default=0.10)
    p.add_argument('--color-dynamic-exclusion', action='store_true')
    p.add_argument('--dynamic-mask-dir', type=Path,
                   help='PNG dynamic-object masks named after image stems')
    p.add_argument('--allow-missing-dynamic-masks', action='store_true',
                   help='attach available masks while retaining unmasked frames')
    p.add_argument('--color-dynamic-mask-margin-px', type=int, default=2)
    p.add_argument('--color-calibration-sigma-multiplier', type=float,
                   default=0.0)
    p.add_argument('--color-max-uncertainty-margin-px', type=int, default=8)
    p.add_argument('--color-min-samples', type=int, default=1,
                   help='demote colours confirmed by fewer surviving camera '
                        'samples than this to unseen (1 keeps all)')
    p.add_argument('--force-images', action='store_true')
    p.add_argument('--force-map', action='store_true')
    p.add_argument('--force-trajectory', action='store_true')
    p.add_argument('--force-calibration', action='store_true')
    p.add_argument('--force-dynamic-masks', action='store_true')
    p.add_argument('--quality-profile', type=Path,
                   help='run alignment, held-out colour, and integrated quality '
                        'checks using this profile')
    p.add_argument('--appearance-planar-roughness', action='store_true',
                   help='include PCA-planar voxel roughness in appearance report')
    p.add_argument('--alignment-diagnostics', action='store_true',
                   help='write signed residual overlays for worst camera views')
    p.add_argument('--alignment-diagnostic-worst-views', type=int, default=10)
    p.add_argument('--alignment-depth-support-radius', type=int, default=0)
    p.add_argument('--alignment-depth-support-min-neighbors', type=int,
                   default=4)
    p.add_argument('--alignment-depth-support-absolute', type=float,
                   default=0.10)
    p.add_argument('--alignment-depth-support-relative', type=float,
                   default=0.01)
    p.add_argument('--alignment-fixed-contours', action='store_true')
    p.add_argument('--alignment-contour-association-distance', type=int,
                   default=0)
    p.add_argument('--alignment-contour-max-points-per-view', type=int,
                   default=50000)
    p.add_argument('--alignment-contour-min-points-per-view', type=int,
                   default=500)
    p.add_argument('--alignment-orientation-max-angle-deg', type=float,
                   default=0.0)
    p.add_argument('--trajectory-report', type=Path,
                   help='metrics.json containing evo.ape.rmse for quality gate')
    p.add_argument('--geometry-report', type=Path,
                   help='map_quality_report.yaml for quality gate')
    p.add_argument('--force-quality', action='store_true',
                   help='rerun quality reports even when outputs are current')
    p.add_argument('--dry-run', action='store_true')
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary = run_pipeline(args)
    if summary['stages']:
        print('completed stages:', ', '.join(summary['stages']))
    else:
        print('everything is up to date; nothing to do')
    print('trajectory:', summary['trajectory'])
    print('coloured map:', summary['colored_map'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
