#!/usr/bin/env python3
"""Deterministic production helpers for LiDAR-camera calibration.

The module deliberately depends on NumPy only.  Geometry projection and image
edge objectives remain in ``evaluate_lidar_camera_alignment.py``; this file
owns search policy, validation partitioning, and local identifiability so they
can be tested without ROS, image codecs, or a real point cloud.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np


PARAMETER_NAMES = ('dt', 'tx', 'ty', 'tz', 'roll', 'pitch', 'yaw')


def downsample_nearest(image: np.ndarray, scale: float) -> np.ndarray:
    """Return a deterministic nearest-neighbour image pyramid level."""
    array = np.asarray(image)
    if array.ndim not in (2, 3):
        raise ValueError(f'image must be HxW or HxWxC, got {array.shape}')
    if not 0.0 < scale <= 1.0:
        raise ValueError('image scale must be in (0, 1]')
    if scale == 1.0:
        return array
    height = max(2, int(round(array.shape[0] * scale)))
    width = max(2, int(round(array.shape[1] * scale)))
    rows = np.minimum(
        (np.arange(height, dtype=np.float64) / scale).astype(np.int64),
        array.shape[0] - 1)
    columns = np.minimum(
        (np.arange(width, dtype=np.float64) / scale).astype(np.int64),
        array.shape[1] - 1)
    return array[rows[:, None], columns[None, :]]


def scale_intrinsics(K: np.ndarray, scale: float) -> np.ndarray:
    """Scale pinhole intrinsics for one image pyramid level."""
    if not 0.0 < scale <= 1.0:
        raise ValueError('intrinsics scale must be in (0, 1]')
    result = np.asarray(K, dtype=np.float64).copy()
    if result.shape != (3, 3):
        raise ValueError(f'K must be 3x3, got {result.shape}')
    result[0, :] *= scale
    result[1, :] *= scale
    result[2, :] = [0.0, 0.0, 1.0]
    return result


def _rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def stratified_view_split(
        stamps: np.ndarray, world_T_bodies: np.ndarray,
        selected: Sequence[int], *, holdout_fraction: float = 0.2,
        spatial_segments: int = 4) -> tuple[list[int], list[int], dict]:
    """Split views across path position and local motion deterministically.

    Spatial strata are equal travelled-distance segments (falling back to
    temporal progress for a stationary path).  Each view is additionally
    labelled ``static``, ``translation``, or ``rotation`` from its local motion.
    Every non-singleton stratum contributes validation views, preventing a
    periodic camera stride from placing the holdout in only one motion regime.
    """
    indices = np.asarray(list(selected), dtype=np.int64)
    stamps = np.asarray(stamps, dtype=np.float64)
    poses = np.asarray(world_T_bodies, dtype=np.float64)
    if indices.size < 2:
        raise ValueError('stratified split needs at least two selected views')
    if poses.shape != (indices.size, 4, 4):
        raise ValueError('world_T_bodies must match selected views')
    if not 0.0 < holdout_fraction < 0.5:
        raise ValueError('holdout_fraction must be in (0, 0.5)')
    if spatial_segments < 1:
        raise ValueError('spatial_segments must be positive')

    translations = poses[:, :3, 3]
    step_translation = np.linalg.norm(
        np.diff(translations, axis=0), axis=1)
    step_rotation = np.asarray([
        _rotation_angle_deg(first, second)
        for first, second in zip(poses, poses[1:])])
    local_translation = np.r_[step_translation[0], step_translation]
    local_rotation = np.r_[step_rotation[0], step_rotation]
    translation_scale = max(float(np.median(
        local_translation[local_translation > 1e-9]))
        if np.any(local_translation > 1e-9) else 0.0, 1e-9)
    rotation_scale = max(float(np.median(
        local_rotation[local_rotation > 1e-6]))
        if np.any(local_rotation > 1e-6) else 0.0, 1e-6)

    motion_labels = []
    for translation, rotation in zip(local_translation, local_rotation):
        if translation < 0.1 * translation_scale and rotation < 0.1 * rotation_scale:
            motion_labels.append('static')
        elif rotation / rotation_scale >= translation / translation_scale:
            motion_labels.append('rotation')
        else:
            motion_labels.append('translation')

    progress = np.r_[0.0, np.cumsum(step_translation)]
    if progress[-1] <= 1e-9:
        duration = max(float(stamps[indices[-1]] - stamps[indices[0]]), 1e-9)
        progress = (stamps[indices] - stamps[indices[0]]) / duration
    else:
        progress /= progress[-1]
    segment_ids = np.minimum(
        (progress * spatial_segments).astype(np.int64), spatial_segments - 1)

    strata: dict[tuple[int, str], list[int]] = {}
    for ordinal, (segment, motion) in enumerate(
            zip(segment_ids, motion_labels)):
        strata.setdefault((int(segment), motion), []).append(ordinal)

    heldout_ordinals: set[int] = set()
    report_strata = []
    for key in sorted(strata):
        members = strata[key]
        count = (min(len(members) - 1,
                     max(1, int(round(len(members) * holdout_fraction))))
                 if len(members) > 1 else 0)
        chosen = []
        if count:
            positions = np.linspace(
                0, len(members) - 1, 2 * count + 1, dtype=np.float64)[1::2]
            chosen = [members[int(round(position))] for position in positions]
            heldout_ordinals.update(chosen)
        report_strata.append({
            'spatial_segment': key[0], 'motion': key[1],
            'views': len(members), 'heldout': len(chosen),
        })
    if not heldout_ordinals:
        heldout_ordinals.add(indices.size // 2)
    if len(heldout_ordinals) >= indices.size:
        heldout_ordinals.remove(max(heldout_ordinals))

    heldout = [int(indices[item]) for item in sorted(heldout_ordinals)]
    train = [int(index) for ordinal, index in enumerate(indices)
             if ordinal not in heldout_ordinals]
    report = {
        'strategy': 'travelled_distance_x_motion',
        'holdout_fraction_requested': holdout_fraction,
        'holdout_fraction_actual': len(heldout) / len(indices),
        'spatial_segments': spatial_segments,
        'strata': report_strata,
    }
    return train, heldout, report


def bounded_coordinate_search(
        objective: Callable[[np.ndarray], float], initial: np.ndarray,
        steps: np.ndarray, bounds: np.ndarray, *, rounds: int = 2,
        maximum_sweeps: int = 100,
        step_multipliers: Sequence[float] = (1.0, 2.0),
        ) -> tuple[np.ndarray, float, dict]:
    """Run deterministic bounded coordinate pattern search from ``initial``."""
    parameters = np.asarray(initial, dtype=np.float64).copy()
    current_steps = np.asarray(steps, dtype=np.float64).copy()
    bounds = np.asarray(bounds, dtype=np.float64)
    if parameters.shape != current_steps.shape or parameters.shape != bounds.shape:
        raise ValueError('initial, steps, and bounds must have matching shapes')
    multipliers = tuple(float(value) for value in step_multipliers)
    if (rounds < 1 or maximum_sweeps < 1 or
            np.any(current_steps <= 0.0) or not multipliers or
            any(value <= 0.0 for value in multipliers)):
        raise ValueError('rounds, sweeps, and steps must be positive')
    if np.any(bounds < 0.0) or np.any(np.abs(parameters) > bounds + 1e-12):
        raise ValueError('initial parameters must lie inside non-negative bounds')
    best_loss = float(objective(parameters))
    history = [{'round': 0, 'loss': best_loss,
                'parameters': parameters.tolist()}]
    evaluations = 1
    sweeps = 0
    for level in range(rounds):
        changed = True
        while changed:
            changed = False
            sweeps += 1
            if sweeps > maximum_sweeps:
                raise RuntimeError('coordinate search exceeded maximum sweeps')
            for axis in range(parameters.size):
                axis_parameters = parameters
                axis_best_parameters = parameters
                axis_best_loss = best_loss
                for multiplier in multipliers:
                    for direction in (-1.0, 1.0):
                        candidate = axis_parameters.copy()
                        candidate[axis] += (
                            direction * multiplier * current_steps[axis])
                        if abs(candidate[axis]) > bounds[axis] + 1e-12:
                            continue
                        loss = float(objective(candidate))
                        evaluations += 1
                        if loss + 1e-12 < axis_best_loss:
                            axis_best_parameters = candidate
                            axis_best_loss = loss
                if axis_best_loss + 1e-12 < best_loss:
                    parameters = axis_best_parameters
                    best_loss = axis_best_loss
                    changed = True
            history.append({
                'round': level + 1, 'sweep': sweeps, 'loss': best_loss,
                'parameters': parameters.tolist(),
            })
        current_steps *= 0.5
    return parameters, best_loss, {
        'evaluations': evaluations, 'sweeps': sweeps,
        'final_steps': current_steps.tolist(),
        'step_multipliers': list(multipliers), 'history': history,
    }


def boundary_axes(parameters: np.ndarray, bounds: np.ndarray,
                  *, tolerance: float | np.ndarray = 1e-9) -> list[str]:
    """Return named parameters touching a nonzero search bound."""
    values = np.asarray(parameters, dtype=np.float64)
    limits = np.asarray(bounds, dtype=np.float64)
    tolerances = np.broadcast_to(
        np.asarray(tolerance, dtype=np.float64), values.shape)
    if np.any(tolerances < 0.0):
        raise ValueError('boundary tolerance must be non-negative')
    return [name for name, value, limit, margin in zip(
        PARAMETER_NAMES, values, limits, tolerances)
        if limit > 0.0 and abs(value) >= limit - margin]


def finite_difference_observability(
        objective: Callable[[np.ndarray], float], parameters: np.ndarray,
        perturbations: np.ndarray, *,
        correlation_pairs: Sequence[tuple[int, int]] = ((0, 1), (0, 2), (0, 3)),
        minimum_curvature: float = 1e-6,
        maximum_condition: float = 1e6,
        maximum_correlation: float = 0.98,
        maximum_stationary_offset: float = 1.0) -> dict:
    """Estimate local curvature, uncertainty, and clock/lever-arm coupling.

    Each axis uses a five-point quadratic fit instead of a single second
    difference.  This is less sensitive to pixel quantisation in the image-edge
    objective and also checks that the fitted stationary point remains local.
    Clock/translation pairs use a 3x3 quadratic surface.  A correlation is only
    reported when that pair is positive definite; an invalid variance is never
    converted into a synthetic perfect correlation.
    """
    values = np.asarray(parameters, dtype=np.float64)
    steps = np.asarray(perturbations, dtype=np.float64)
    if values.shape != (7,) or steps.shape != (7,) or np.any(steps <= 0.0):
        raise ValueError('observability expects seven positive perturbations')
    if maximum_stationary_offset <= 0.0:
        raise ValueError('maximum_stationary_offset must be positive')
    base = float(objective(values))
    hessian = np.zeros((7, 7), dtype=np.float64)
    evaluations = 1
    finite = np.isfinite(base)
    axis_fits = []
    offsets = np.arange(-2.0, 3.0)
    axis_design = np.column_stack((offsets ** 2, offsets,
                                   np.ones(offsets.size)))
    for axis in range(7):
        losses = []
        for offset in offsets:
            if offset == 0.0:
                losses.append(base)
                continue
            candidate = values.copy()
            candidate[axis] += offset * steps[axis]
            losses.append(float(objective(candidate)))
            evaluations += 1
        axis_finite = bool(np.all(np.isfinite(losses)))
        finite &= axis_finite
        if axis_finite:
            coefficients = np.linalg.lstsq(
                axis_design, np.asarray(losses), rcond=None)[0]
            predicted = axis_design @ coefficients
            residual = float(np.sum((np.asarray(losses) - predicted) ** 2))
            total = float(np.sum((np.asarray(losses) - np.mean(losses)) ** 2))
            fit_quality = 1.0 - residual / total if total > 1e-15 else 1.0
            curvatures, stationary_offsets = [], []
            for radius in (1, 2):
                minus_loss = losses[2 - radius]
                plus_loss = losses[2 + radius]
                curvature_at_radius = float(
                    (plus_loss + minus_loss - 2.0 * base) / radius ** 2)
                gradient_at_radius = float(
                    (plus_loss - minus_loss) / (2.0 * radius))
                curvatures.append(curvature_at_radius)
                stationary_offsets.append(
                    float(-gradient_at_radius / curvature_at_radius)
                    if abs(curvature_at_radius) > 1e-15 else None)
            curvature = float(np.median(curvatures))
            finite_offsets = [value for value in stationary_offsets
                              if value is not None and np.isfinite(value)]
            stationary_offset = (float(np.median(finite_offsets))
                                 if finite_offsets else None)
        else:
            curvature, stationary_offset, fit_quality = np.nan, None, np.nan
            curvatures, stationary_offsets = [np.nan, np.nan], [None, None]
        hessian[axis, axis] = curvature
        axis_fits.append({
            'axis': PARAMETER_NAMES[axis],
            'curvature': curvature,
            'curvature_at_one_and_two_steps': curvatures,
            'curvature_stable': bool(
                all(value > minimum_curvature for value in curvatures)),
            'stationary_offset_steps': stationary_offset,
            'stationary_offset_at_one_and_two_steps': stationary_offsets,
            'fit_r_squared': fit_quality,
            'losses_at_offsets_minus2_to_plus2': losses,
        })

    correlations = []
    pair_fits = []
    pair_offsets = (-1.0, 0.0, 1.0)
    for first, second in correlation_pairs:
        rows, losses = [], []
        for first_offset in pair_offsets:
            for second_offset in pair_offsets:
                if first_offset == 0.0 and second_offset == 0.0:
                    loss = base
                else:
                    candidate = values.copy()
                    candidate[first] += first_offset * steps[first]
                    candidate[second] += second_offset * steps[second]
                    loss = float(objective(candidate))
                    evaluations += 1
                rows.append((first_offset ** 2, second_offset ** 2,
                             first_offset * second_offset, first_offset,
                             second_offset, 1.0))
                losses.append(loss)
        pair_finite = bool(np.all(np.isfinite(losses)))
        finite &= pair_finite
        if pair_finite:
            design = np.asarray(rows, dtype=np.float64)
            coefficients = np.linalg.lstsq(
                design, np.asarray(losses), rcond=None)[0]
            cross = float(coefficients[2])
            pair_hessian = np.array([
                [2.0 * coefficients[0], cross],
                [cross, 2.0 * coefficients[1]],
            ])
            determinant = float(np.linalg.det(pair_hessian))
            pair_positive = bool(
                pair_hessian[0, 0] > minimum_curvature and
                pair_hessian[1, 1] > minimum_curvature and
                determinant > minimum_curvature ** 2)
            correlation = (float(-cross / np.sqrt(
                pair_hessian[0, 0] * pair_hessian[1, 1]))
                           if pair_positive else None)
        else:
            cross, determinant, pair_positive, correlation = (
                np.nan, np.nan, False, None)
        hessian[first, second] = hessian[second, first] = cross
        correlations.append({
            'first': PARAMETER_NAMES[first],
            'second': PARAMETER_NAMES[second],
            'correlation': correlation,
        })
        pair_fits.append({
            'first': PARAMETER_NAMES[first],
            'second': PARAMETER_NAMES[second],
            'positive_definite': pair_positive,
            'determinant': determinant,
        })

    eigenvalues = np.linalg.eigvalsh(hessian) if finite else np.full(7, np.nan)
    positive = eigenvalues[eigenvalues > minimum_curvature]
    condition = (float(positive[-1] / positive[0])
                 if positive.size == 7 else float('inf'))
    hessian_positive = bool(finite and positive.size == 7)
    if hessian_positive:
        covariance_normalized = np.linalg.inv(hessian) * max(base, 1e-6)
        covariance = (np.diag(steps) @ covariance_normalized @
                      np.diag(steps))
        uncertainties = np.sqrt(np.diag(covariance)).tolist()
    else:
        uncertainties = [None] * 7
    defined_correlations = [
        abs(item['correlation']) for item in correlations
        if item['correlation'] is not None]
    max_correlation = (max(defined_correlations)
                       if defined_correlations else None)
    pairs_observable = all(item['positive_definite'] for item in pair_fits)
    curvature_stable = all(item['curvature_stable'] for item in axis_fits)
    stationary = all(
        item['stationary_offset_steps'] is not None and
        abs(item['stationary_offset_steps']) <= maximum_stationary_offset
        for item in axis_fits)
    observable = bool(
        hessian_positive and curvature_stable and pairs_observable and stationary and
        condition <= maximum_condition and max_correlation is not None and
        max_correlation <= maximum_correlation)
    reasons = []
    if not finite:
        reasons.append('non_finite_local_objective')
    if positive.size != 7:
        reasons.append('insufficient_positive_curvature')
    if not curvature_stable:
        reasons.append('unstable_multiscale_curvature')
    if not pairs_observable:
        reasons.append('unobservable_time_translation_pair')
    if not stationary:
        reasons.append('stationary_point_outside_local_neighborhood')
    if condition > maximum_condition:
        reasons.append('ill_conditioned')
    if (max_correlation is not None and
            max_correlation > maximum_correlation):
        reasons.append('time_translation_correlation')
    return {
        'observable': observable,
        'rejection_reasons': reasons,
        'evaluations': evaluations,
        'base_loss': base,
        'perturbations_dt_s_xyz_m_rpy_rad': steps.tolist(),
        'standardized_hessian': hessian.tolist(),
        'eigenvalues': eigenvalues.tolist(),
        'condition_number': condition,
        'axis_quadratic_fits': axis_fits,
        'pair_quadratic_fits': pair_fits,
        'uncertainty_dt_s_xyz_m_rpy_rad': uncertainties,
        'time_translation_correlations': correlations,
        'maximum_abs_time_translation_correlation': max_correlation,
    }
