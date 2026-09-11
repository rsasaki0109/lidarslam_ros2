#!/usr/bin/env python3
"""Ground-truth-independent, report-only v42 pose-graph feasibility audit."""

from __future__ import annotations

import argparse
from collections import OrderedDict
import csv
import hashlib
import json
import math
from pathlib import Path
import resource
import time
from typing import Any, Iterable

import numpy as np


class ContractError(ValueError):
    """Raised when an input violates the frozen v42 contract."""


class MemoryBudgetError(RuntimeError):
    """Raised before continuing work outside a frozen memory budget."""


def canonical_json(payload: Any) -> str:
    """Serialize a finite payload deterministically."""
    return json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(payload: Any) -> str:
    return hashlib.sha256(canonical_json(payload).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _finite_float(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def load_contract(path: Path) -> tuple[dict[str, Any], str]:
    contract = json.loads(path.read_text(encoding='utf-8'))
    required = {
        'schema_version', 'contract_id', 'frames', 'anchor', 'candidate',
        'submap', 'descriptor', 'registration', 'sequence', 'deduplication',
        'constraint', 'memory', 'decision',
    }
    missing = sorted(required - set(contract))
    if missing:
        raise ContractError(f'contract missing keys: {missing}')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported contract schema_version')
    validate_frame_contract(contract['frames'])
    offsets = [int(value) for value in contract['sequence']['offsets']]
    if not offsets or offsets != sorted(set(offsets)) or 0 not in offsets:
        raise ContractError('sequence offsets must be unique, sorted, and contain zero')
    if offsets != [-value for value in reversed(offsets)]:
        raise ContractError('sequence offsets must be symmetric')
    if int(contract['candidate']['maximum_geometry_verifications']) <= 0:
        raise ContractError('maximum_geometry_verifications must be positive')
    if float(contract['memory']['maximum_rss_mib']) <= 0.0:
        raise ContractError('maximum_rss_mib must be positive')
    return contract, sha256_file(path)


def validate_frame_contract(frames: dict[str, Any]) -> None:
    if frames.get('world') != 'map' or frames.get('body') != 'base_link':
        raise ContractError('v42 frame identity must be map <- base_link')


def quaternion_rotation(values: Iterable[float]) -> np.ndarray:
    qx, qy, qz, qw = (float(value) for value in values)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ContractError('pose has an invalid quaternion')
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def pose_matrix(pose: Iterable[float]) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = (float(value) for value in pose)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_rotation((qx, qy, qz, qw))
    transform[:3, 3] = (x, y, z)
    return transform


def load_states(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load exact Voxel-SLAM timestamps and map-from-body poses."""
    stamps: list[float] = []
    transforms: list[np.ndarray] = []
    previous = -math.inf
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) < 8:
            raise ContractError(f'{path}:{line_number}: expected at least 8 fields')
        try:
            values = [float(value) for value in fields[:8]]
        except ValueError as error:
            raise ContractError(f'{path}:{line_number}: non-numeric pose') from error
        if not all(math.isfinite(value) for value in values):
            raise ContractError(f'{path}:{line_number}: non-finite pose')
        if values[0] <= previous:
            raise ContractError(f'{path}:{line_number}: timestamps are not increasing')
        stamps.append(values[0])
        transforms.append(pose_matrix(values[1:]))
        previous = values[0]
    if not stamps:
        raise ContractError(f'{path}: no states')
    return np.asarray(stamps, dtype=np.float64), np.stack(transforms)


def indexed_chunks(directory: Path, expected: int) -> list[Path]:
    chunks = sorted(directory.glob('*.pcd'), key=lambda path: int(path.stem))
    try:
        indices = [int(path.stem) for path in chunks]
    except ValueError as error:
        raise ContractError(f'{directory}: non-numeric PCD chunk name') from error
    if indices != list(range(expected)):
        raise ContractError(f'{directory}: chunks are not contiguous for {expected} states')
    return chunks


def select_anchor_indices(transforms: np.ndarray, minimum_translation_m: float) -> list[int]:
    if len(transforms) == 0:
        return []
    threshold = float(minimum_translation_m)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ContractError('anchor translation threshold must be positive')
    selected = [0]
    for index in range(1, len(transforms)):
        distance = np.linalg.norm(
            transforms[index, :3, 3] - transforms[selected[-1], :3, 3])
        if distance >= threshold:
            selected.append(index)
    return selected


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    value = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return value / 1024.0


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.peak_rss_mib = current_rss_mib()

    def check(self, label: str) -> None:
        rss = current_rss_mib()
        self.peak_rss_mib = max(self.peak_rss_mib, rss)
        if rss > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS {rss:.3f} MiB exceeds {self.maximum_rss_mib:.3f} MiB at {label}')


def read_binary_xyzi_pcd(path: Path) -> np.ndarray:
    """Read one strict packed float32 XYZI binary PCD without whole-file copies."""
    fields: dict[str, list[str]] = {}
    with path.open('rb') as stream:
        for _ in range(64):
            raw = stream.readline()
            if not raw:
                raise ContractError(f'{path}: incomplete PCD header')
            parts = raw.decode('ascii', errors='strict').split()
            if parts:
                fields[parts[0]] = parts[1:]
            if raw.strip() == b'DATA binary':
                offset = stream.tell()
                break
        else:
            raise ContractError(f'{path}: PCD header is too long')
    expected = (
        ['x', 'y', 'z', 'intensity'], ['4'] * 4, ['F'] * 4, ['1'] * 4, ['binary'])
    actual = tuple(fields.get(key) for key in ('FIELDS', 'SIZE', 'TYPE', 'COUNT', 'DATA'))
    if actual != expected or 'POINTS' not in fields:
        raise ContractError(f'{path}: expected packed float32 XYZI binary PCD')
    points = int(fields['POINTS'][0])
    if path.stat().st_size - offset != points * 16:
        raise ContractError(f'{path}: PCD byte count differs from POINTS')
    values = np.fromfile(path, dtype='<f4', count=points * 4, offset=offset)
    if values.size != points * 4:
        raise ContractError(f'{path}: truncated PCD payload')
    return values.reshape(points, 4)


def transform_xyz(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def voxel_downsample_xyzi(points: np.ndarray, voxel_size_m: float,
                          maximum_points: int) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 4), dtype=np.float32)
    keys = np.floor(points[:, :3] / float(voxel_size_m)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    result = points[np.sort(first)]
    limit = int(maximum_points)
    if len(result) > limit:
        keep = np.linspace(0, len(result) - 1, limit, dtype=np.int64)
        result = result[keep]
    return np.asarray(result, dtype=np.float32)


class StreamingSubmapLoader:
    """Build bounded anchor-local submaps while retaining only a tiny chunk LRU."""

    def __init__(self, transforms: np.ndarray, anchors: list[int], chunks: list[Path],
                 config: dict[str, Any], memory: MemoryGuard) -> None:
        self.transforms = transforms
        self.anchors = anchors
        self.chunks = chunks
        self.config = config
        self.memory = memory
        self.cache: OrderedDict[int, np.ndarray] = OrderedDict()
        self.cache_bytes = 0
        self.peak_cache_bytes = 0
        self.peak_submap_bytes = 0

    def _body_chunk(self, state_index: int) -> np.ndarray:
        if state_index in self.cache:
            points = self.cache.pop(state_index)
            self.cache[state_index] = points
            return points
        points = read_binary_xyzi_pcd(self.chunks[state_index])
        xyz = points[:, :3]
        ranges = np.linalg.norm(xyz, axis=1)
        finite = np.isfinite(points).all(axis=1)
        keep = finite & (ranges >= float(self.config['minimum_range_m'])) & (
            ranges <= float(self.config['maximum_range_m']))
        points = np.asarray(points[keep], dtype=np.float32).copy()
        maximum_cache = int(self.config['maximum_chunk_cache_bytes'])
        if points.nbytes > maximum_cache:
            raise MemoryBudgetError('one filtered chunk exceeds maximum_chunk_cache_bytes')
        entries = int(self.config['chunk_cache_entries'])
        while self.cache and (len(self.cache) >= entries or
                              self.cache_bytes + points.nbytes > maximum_cache):
            _, evicted = self.cache.popitem(last=False)
            self.cache_bytes -= evicted.nbytes
        self.cache[state_index] = points
        self.cache_bytes += points.nbytes
        self.peak_cache_bytes = max(self.peak_cache_bytes, self.cache_bytes)
        self.memory.check('chunk_load')
        return points

    def build(self, anchor_ordinal: int) -> np.ndarray:
        radius = int(self.config['anchor_radius'])
        if not 0 <= anchor_ordinal < len(self.anchors):
            raise ContractError('submap anchor ordinal is out of range')
        center_state = self.anchors[anchor_ordinal]
        center_from_world = np.linalg.inv(self.transforms[center_state])
        clouds: list[np.ndarray] = []
        raw_points = 0
        begin = max(0, anchor_ordinal - radius)
        end = min(len(self.anchors), anchor_ordinal + radius + 1)
        for neighbor_ordinal in range(begin, end):
            state_index = self.anchors[neighbor_ordinal]
            body = self._body_chunk(state_index)
            raw_points += len(body)
            if raw_points > int(self.config['maximum_raw_points']):
                raise MemoryBudgetError('submap exceeds maximum_raw_points')
            center_from_body = center_from_world @ self.transforms[state_index]
            xyz = transform_xyz(body[:, :3].astype(np.float64), center_from_body)
            cloud = np.column_stack((xyz, body[:, 3])).astype(np.float32)
            clouds.append(cloud)
        merged = np.concatenate(clouds, axis=0) if clouds else np.empty((0, 4), np.float32)
        result = voxel_downsample_xyzi(
            merged, float(self.config['voxel_size_m']), int(self.config['maximum_points']))
        self.peak_submap_bytes = max(self.peak_submap_bytes, result.nbytes)
        if result.nbytes > int(self.config['maximum_submap_bytes']):
            raise MemoryBudgetError('downsampled submap exceeds maximum_submap_bytes')
        self.memory.check('submap_build')
        return result


def scan_context_descriptor(points: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    rings = int(config['rings'])
    sectors = int(config['sectors'])
    descriptor = np.zeros((2, rings, sectors), dtype=np.float32)
    if len(points) == 0:
        return descriptor
    xy = points[:, :2].astype(np.float64)
    radius = np.linalg.norm(xy, axis=1)
    maximum_radius = float(config['maximum_radius_m'])
    valid = np.isfinite(radius) & (radius > 0.0) & (radius <= maximum_radius)
    if not np.any(valid):
        return descriptor
    radius = radius[valid]
    angle = np.mod(np.arctan2(xy[valid, 1], xy[valid, 0]), 2.0 * math.pi)
    ring = np.minimum((radius / maximum_radius * rings).astype(np.int64), rings - 1)
    sector = np.minimum((angle / (2.0 * math.pi) * sectors).astype(np.int64), sectors - 1)
    counts = np.zeros((rings, sectors), dtype=np.int32)
    np.add.at(counts, (ring, sector), 1)
    maximum_count = int(np.max(counts))
    if maximum_count:
        descriptor[0] = np.log1p(counts) / math.log1p(maximum_count)
    low = float(config['minimum_height_m'])
    high = float(config['maximum_height_m'])
    height = np.full((rings, sectors), low, dtype=np.float32)
    clipped = np.clip(points[valid, 2], low, high).astype(np.float32)
    np.maximum.at(height, (ring, sector), clipped)
    occupied = counts > 0
    descriptor[1, occupied] = (height[occupied] - low) / (high - low)
    return descriptor


class DescriptorIndex:
    def __init__(self, spectra: np.ndarray, norms: np.ndarray, sectors: int,
                 point_counts: list[int]) -> None:
        self.spectra = spectra
        self.norms = norms
        self.sectors = sectors
        self.point_counts = point_counts

    def similarity(self, first: int, second: int) -> float:
        denominator = float(self.norms[first] * self.norms[second])
        if denominator <= 1e-12:
            return 0.0
        product = np.conj(self.spectra[first]) * self.spectra[second]
        correlation = np.fft.irfft(product, n=self.sectors, axis=-1).sum(axis=(0, 1))
        return float(np.clip(np.max(correlation) / denominator, 0.0, 1.0))


def build_descriptor_index(loader: StreamingSubmapLoader,
                           config: dict[str, Any]) -> DescriptorIndex:
    count = len(loader.anchors)
    rings = int(config['rings'])
    sectors = int(config['sectors'])
    spectra = np.empty((count, 2, rings, sectors // 2 + 1), dtype=np.complex64)
    norms = np.empty(count, dtype=np.float32)
    point_counts: list[int] = []
    for anchor in range(count):
        submap = loader.build(anchor)
        point_counts.append(int(len(submap)))
        descriptor = scan_context_descriptor(submap, config)
        spectra[anchor] = np.fft.rfft(descriptor, axis=-1).astype(np.complex64)
        norms[anchor] = float(np.linalg.norm(descriptor))
        if anchor % 64 == 0:
            loader.memory.check('descriptor_index')
    loader.memory.check('descriptor_index_complete')
    return DescriptorIndex(spectra, norms, sectors, point_counts)


def cumulative_travel_m(transforms: np.ndarray, anchors: list[int]) -> np.ndarray:
    positions = transforms[anchors, :3, 3]
    if len(positions) <= 1:
        return np.zeros(len(positions), dtype=np.float64)
    increments = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(increments)))


def group_episodes(indices: Iterable[int], maximum_gap: int) -> list[list[int]]:
    episodes: list[list[int]] = []
    for index in sorted(int(value) for value in indices):
        if not episodes or index - episodes[-1][-1] > maximum_gap:
            episodes.append([index])
        else:
            episodes[-1].append(index)
    return episodes


def _sequence_descriptor_candidate(
        query: int, episode: list[int], direction: int, offsets: list[int],
        descriptors: DescriptorIndex, travel: np.ndarray, positions: np.ndarray,
        minimum_separation: float) -> list[dict[str, Any]]:
    candidates = []
    count = len(positions)
    for source in episode:
        pairs = [(source + direction*offset, query + offset) for offset in offsets]
        if any(not (0 <= first < count and 0 <= second < count and first < second)
               for first, second in pairs):
            continue
        if any(travel[second] - travel[first] < minimum_separation
               for first, second in pairs):
            continue
        similarities = [descriptors.similarity(first, second) for first, second in pairs]
        candidates.append({
            'source_anchor': int(source),
            'target_anchor': int(query),
            'direction': 'forward' if direction == 1 else 'reverse',
            'sequence_similarity': float(min(similarities)),
            'median_similarity': float(np.median(similarities)),
            'offset_similarities': [float(value) for value in similarities],
            'estimated_distance_m': float(np.linalg.norm(positions[source] - positions[query])),
        })
    return candidates


def generate_candidates(transforms: np.ndarray, anchors: list[int],
                        descriptors: DescriptorIndex, contract: dict[str, Any]
                        ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    config = contract['candidate']
    offsets = [int(value) for value in contract['sequence']['offsets']]
    positions = transforms[anchors, :3, 3]
    travel = cumulative_travel_m(transforms, anchors)
    records: list[dict[str, Any]] = []
    qualified: list[dict[str, Any]] = []
    minimum_separation = float(config['minimum_travel_separation_m'])
    radius = float(config['search_radius_m'])
    gap = int(config['episode_gap_anchors'])
    stride = int(config['query_stride'])
    maximum_episodes = int(config['maximum_episodes_per_query'])
    for query in range(0, len(anchors), stride):
        distances = np.linalg.norm(positions[:query] - positions[query], axis=1)
        eligible = np.flatnonzero(
            (travel[query] - travel[:query] >= minimum_separation) & (distances <= radius))
        if not len(eligible):
            continue
        episodes = group_episodes(eligible, gap)
        if len(episodes) > maximum_episodes:
            raise MemoryBudgetError(
                f'query {query} has {len(episodes)} episodes, limit {maximum_episodes}')
        alternatives = []
        for episode_index, episode in enumerate(episodes):
            proposals = []
            proposals.extend(_sequence_descriptor_candidate(
                query, episode, 1, offsets, descriptors, travel, positions,
                minimum_separation))
            proposals.extend(_sequence_descriptor_candidate(
                query, episode, -1, offsets, descriptors, travel, positions,
                minimum_separation))
            if not proposals:
                continue
            proposals.sort(key=lambda item: (
                -item['sequence_similarity'], -item['median_similarity'],
                item['estimated_distance_m'], item['source_anchor'], item['direction']))
            best_episode = dict(proposals[0])
            best_episode['episode_index'] = episode_index
            best_episode['episode_begin'] = int(episode[0])
            best_episode['episode_end'] = int(episode[-1])
            alternatives.append(best_episode)
        alternatives.sort(key=lambda item: (
            -item['sequence_similarity'], -item['median_similarity'],
            item['estimated_distance_m'], item['source_anchor'], item['direction']))
        if not alternatives:
            records.append({
                'target_anchor': int(query),
                'disposition': 'REJECT_INCOMPLETE_SEQUENCE',
                'episode_count': len(episodes),
                'alternatives': [],
            })
            continue
        best = alternatives[0]
        second_score = alternatives[1]['sequence_similarity'] if len(alternatives) > 1 else 0.0
        margin = float(best['sequence_similarity'] - second_score)
        record = dict(best)
        record['episode_count'] = len(episodes)
        record['descriptor_margin'] = margin
        record['alternatives'] = alternatives
        if best['sequence_similarity'] < float(config['minimum_sequence_descriptor_similarity']):
            record['disposition'] = 'REJECT_DESCRIPTOR_SIMILARITY'
        elif margin < float(config['minimum_sequence_descriptor_margin']):
            record['disposition'] = 'REJECT_DESCRIPTOR_AMBIGUITY'
        else:
            record['disposition'] = 'QUALIFIED_FOR_GEOMETRY'
            qualified.append(record)
        records.append(record)
    qualified.sort(key=lambda item: (item['target_anchor'], item['source_anchor'], item['direction']))
    return records, qualified


def nearest_distances_indices(source: np.ndarray, target: np.ndarray
                              ) -> tuple[np.ndarray, np.ndarray]:
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError as error:
        if len(source) * len(target) > 1_000_000:
            raise ContractError('scipy is required for bounded real-data registration') from error
        delta = source[:, None, :] - target[None, :, :]
        squared = np.sum(delta * delta, axis=2)
        indices = np.argmin(squared, axis=1)
        return np.sqrt(squared[np.arange(len(source)), indices]), indices
    distances, indices = cKDTree(target).query(source, k=1, workers=1)
    return distances.astype(np.float64, copy=False), indices.astype(np.int64, copy=False)


def rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    if len(source) != len(target) or len(source) < 3:
        raise ContractError('rigid transform needs at least three point pairs')
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u_matrix, _, vh_matrix = np.linalg.svd(covariance)
    rotation = vh_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vh_matrix[-1] *= -1.0
        rotation = vh_matrix.T @ u_matrix.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def harmonic_overlap(first: float, second: float) -> float:
    return 0.0 if first + second <= 0.0 else 2.0 * first * second / (first + second)


def register_clouds(source: np.ndarray, target: np.ndarray, initial: np.ndarray,
                    config: dict[str, Any]) -> dict[str, Any]:
    """Constrained deterministic ICP from source anchor frame to target frame."""
    if len(source) < 3 or len(target) < 3:
        return {'passed': False, 'reasons': ['insufficient_points']}
    source_xyz = source[:, :3].astype(np.float64)
    target_xyz = target[:, :3].astype(np.float64)
    correction = np.eye(4, dtype=np.float64)
    aligned = transform_xyz(source_xyz, initial)
    target_tree_points = target_xyz
    reasons: list[str] = []
    iterations = 0
    for iteration in range(int(config['maximum_iterations'])):
        distances, indices = nearest_distances_indices(aligned, target_tree_points)
        valid = np.flatnonzero(distances <= float(config['maximum_correspondence_m']))
        ratio = len(valid) / max(1, min(len(source_xyz), len(target_xyz)))
        if (len(valid) < int(config['minimum_correspondences']) or
                ratio < float(config['minimum_correspondence_ratio'])):
            reasons.append('insufficient_correspondence_support')
            break
        order = valid[np.argsort(distances[valid], kind='stable')]
        keep_count = max(
            int(config['minimum_correspondences']),
            int(math.floor(len(order) * float(config['trim_fraction']))))
        keep = order[:min(len(order), keep_count)]
        delta = rigid_transform(aligned[keep], target_xyz[indices[keep]])
        aligned = transform_xyz(aligned, delta)
        correction = delta @ correction
        iterations = iteration + 1
        if (np.linalg.norm(delta[:3, 3]) <= float(config['convergence_translation_m']) and
                rotation_angle_deg(delta[:3, :3]) <=
                float(config['convergence_rotation_deg'])):
            break
    forward, _ = nearest_distances_indices(aligned, target_xyz)
    reverse, _ = nearest_distances_indices(target_xyz, aligned)
    overlap_distance = float(config['overlap_distance_m'])
    forward_overlap = float(np.mean(forward <= overlap_distance))
    reverse_overlap = float(np.mean(reverse <= overlap_distance))
    mutual = harmonic_overlap(forward_overlap, reverse_overlap)
    support = forward[forward <= float(config['maximum_correspondence_m'])]
    rmse = float(np.sqrt(np.mean(support * support))) if len(support) else math.inf
    p90 = float(np.percentile(support, 90)) if len(support) else math.inf
    correction_translation = float(np.linalg.norm(correction[:3, 3]))
    correction_rotation = rotation_angle_deg(correction[:3, :3])
    if mutual < float(config['minimum_pair_mutual_overlap']):
        reasons.append('mutual_overlap')
    if rmse > float(config['maximum_support_rmse_m']):
        reasons.append('support_rmse')
    if p90 > float(config['maximum_support_p90_m']):
        reasons.append('support_p90')
    if correction_translation > float(config['maximum_correction_translation_m']):
        reasons.append('correction_translation')
    if correction_rotation > float(config['maximum_correction_rotation_deg']):
        reasons.append('correction_rotation')
    target_from_source = correction @ initial
    return {
        'passed': not reasons,
        'reasons': sorted(set(reasons)),
        'iterations': iterations,
        'source_points': int(len(source_xyz)),
        'target_points': int(len(target_xyz)),
        'forward_overlap': forward_overlap,
        'reverse_overlap': reverse_overlap,
        'mutual_overlap': mutual,
        'support_count': int(len(support)),
        'support_rmse_m': _finite_float(rmse),
        'support_p90_m': _finite_float(p90),
        'correction_translation_m': correction_translation,
        'correction_rotation_deg': correction_rotation,
        'correction': correction,
        'target_from_source': target_from_source,
    }


def validate_covariance(diagonal: Iterable[float], maximum_condition: float) -> list[float]:
    values = np.asarray(list(diagonal), dtype=np.float64)
    if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0.0):
        raise ContractError('constraint covariance must be finite positive diagonal 6x6')
    if float(np.max(values) / np.min(values)) > float(maximum_condition):
        raise ContractError('constraint covariance exceeds condition limit')
    return [float(value) for value in values]


def validate_se3(transform: np.ndarray) -> None:
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ContractError('constraint transform must be finite 4x4')
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ContractError('constraint transform has an invalid homogeneous row')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ContractError('constraint rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ContractError('constraint rotation determinant is not one')


def pose_graph_measurement(target_from_source: np.ndarray) -> np.ndarray:
    """Return source-from-target (X_source^-1 X_target) for a pose-graph edge."""
    validate_se3(target_from_source)
    measurement = np.linalg.inv(target_from_source)
    validate_se3(measurement)
    return measurement


def register_candidate(candidate: dict[str, Any], loader: StreamingSubmapLoader,
                       contract: dict[str, Any]) -> dict[str, Any]:
    offsets = [int(value) for value in contract['sequence']['offsets']]
    direction = 1 if candidate['direction'] == 'forward' else -1
    source_center = int(candidate['source_anchor'])
    target_center = int(candidate['target_anchor'])
    pairs = []
    center_result: dict[str, Any] | None = None
    global_corrections: list[tuple[int, np.ndarray]] = []
    for offset in offsets:
        source = source_center + direction*offset
        target = target_center + offset
        source_cloud = loader.build(source)
        target_cloud = loader.build(target)
        source_state = loader.anchors[source]
        target_state = loader.anchors[target]
        initial = np.linalg.inv(loader.transforms[target_state]) @ loader.transforms[source_state]
        registration = register_clouds(
            source_cloud, target_cloud, initial, contract['registration'])
        correction = registration.pop('correction', None)
        target_from_source = registration.pop('target_from_source', None)
        if correction is not None:
            global_correction = (
                loader.transforms[target_state] @ correction @
                np.linalg.inv(loader.transforms[target_state]))
            global_corrections.append((offset, global_correction))
        if target_from_source is not None:
            registration['target_from_source_matrix'] = target_from_source.tolist()
        registration.update({
            'offset': offset,
            'source_anchor': source,
            'target_anchor': target,
        })
        pairs.append(registration)
        if offset == 0:
            center_result = registration
        del source_cloud, target_cloud
        loader.memory.check('sequence_registration')
    if center_result is None:
        raise ContractError('sequence offsets omitted center result')
    center_global = next((value for offset, value in global_corrections if offset == 0), None)
    translation_spread = math.inf
    rotation_spread = math.inf
    if center_global is not None and len(global_corrections) == len(offsets):
        deltas = [np.linalg.inv(center_global) @ value for _, value in global_corrections]
        translation_spread = max(float(np.linalg.norm(delta[:3, 3])) for delta in deltas)
        rotation_spread = max(rotation_angle_deg(delta[:3, :3]) for delta in deltas)
    overlaps = [float(item.get('mutual_overlap', 0.0)) for item in pairs]
    reasons = []
    if not all(bool(item.get('passed')) for item in pairs):
        reasons.append('pair_gate')
    if float(center_result.get('mutual_overlap', 0.0)) < float(
            contract['sequence']['minimum_center_mutual_overlap']):
        reasons.append('center_overlap')
    if min(overlaps, default=0.0) < float(
            contract['sequence']['minimum_sequence_mutual_overlap']):
        reasons.append('sequence_overlap')
    if translation_spread > float(
            contract['sequence']['maximum_global_correction_translation_spread_m']):
        reasons.append('global_translation_spread')
    if rotation_spread > float(
            contract['sequence']['maximum_global_correction_rotation_spread_deg']):
        reasons.append('global_rotation_spread')
    result: dict[str, Any] = {
        'passed': not reasons,
        'reasons': sorted(set(reasons)),
        'pairs': pairs,
        'minimum_mutual_overlap': min(overlaps, default=0.0),
        'median_mutual_overlap': float(np.median(overlaps)) if overlaps else 0.0,
        'global_correction_translation_spread_m': _finite_float(translation_spread),
        'global_correction_rotation_spread_deg': _finite_float(rotation_spread),
    }
    if not reasons:
        target_from_source = np.asarray(
            center_result['target_from_source_matrix'], dtype=np.float64)
        measurement = pose_graph_measurement(target_from_source)
        covariance_config = contract['constraint']
        covariance = validate_covariance(
            [float(covariance_config['translation_variance_m2'])] * 3 +
            [float(covariance_config['rotation_variance_rad2'])] * 3,
            float(covariance_config['maximum_covariance_condition']))
        result['constraint'] = {
            'source_anchor': source_center,
            'target_anchor': target_center,
            'frame': contract['frames']['world'],
            'measurement_convention': 'source_from_target=X_source^-1*X_target',
            'measurement_matrix': measurement.tolist(),
            'covariance_diagonal': covariance,
            'quality': {
                'sequence_descriptor_similarity': candidate['sequence_similarity'],
                'minimum_mutual_overlap': result['minimum_mutual_overlap'],
                'median_mutual_overlap': result['median_mutual_overlap'],
                'center_support_rmse_m': center_result['support_rmse_m'],
            },
        }
    return result


def deduplicate_constraints(records: list[dict[str, Any]], index_window: int
                            ) -> list[dict[str, Any]]:
    accepted = [record for record in records if record.get('geometry', {}).get('passed')]
    accepted.sort(key=lambda record: (
        -record['geometry']['minimum_mutual_overlap'],
        record['geometry']['pairs'][len(record['geometry']['pairs']) // 2]
        .get('support_rmse_m', math.inf),
        -record['sequence_similarity'], record['source_anchor'], record['target_anchor']))
    selected: list[dict[str, Any]] = []
    for record in accepted:
        duplicate = next((other for other in selected
                          if abs(record['source_anchor'] - other['source_anchor']) <= index_window
                          and abs(record['target_anchor'] - other['target_anchor']) <= index_window), None)
        if duplicate is None:
            record['disposition'] = 'ACCEPT_VERIFIED_CONSTRAINT'
            selected.append(record)
        else:
            record['disposition'] = 'REJECT_DUPLICATE_CONSTRAINT'
            record['duplicate_of'] = [duplicate['source_anchor'], duplicate['target_anchor']]
    constraints = [record['geometry']['constraint'] for record in selected]
    constraints.sort(key=lambda item: (item['source_anchor'], item['target_anchor']))
    pairs = [(item['source_anchor'], item['target_anchor']) for item in constraints]
    if len(pairs) != len(set(pairs)):
        raise ContractError('duplicate canonical constraint pair survived')
    return constraints


def graph_connected(node_count: int, edges: Iterable[tuple[int, int]]) -> bool:
    if node_count <= 0:
        return False
    adjacency = [[] for _ in range(node_count)]
    for first, second in edges:
        if not (0 <= first < node_count and 0 <= second < node_count):
            raise ContractError('graph edge endpoint is out of range')
        adjacency[first].append(second)
        adjacency[second].append(first)
    seen = {0}
    stack = [0]
    while stack:
        node = stack.pop()
        for neighbor in adjacency[node]:
            if neighbor not in seen:
                seen.add(neighbor)
                stack.append(neighbor)
    return len(seen) == node_count


def load_legacy_edges(path: Path | None) -> list[tuple[int, int]]:
    if path is None:
        return []
    rows = list(csv.DictReader(path.open(newline='', encoding='utf-8')))
    edges = []
    for row in rows:
        source, target = int(row['from']), int(row['to'])
        if source >= target:
            raise ContractError(f'{path}: legacy edge direction is not historical->query')
        edges.append((source, target))
    return sorted(set(edges))


def legacy_survivors(constraints: list[dict[str, Any]], legacy: list[tuple[int, int]],
                     index_window: int) -> list[dict[str, Any]]:
    survivors = []
    for constraint in constraints:
        source = int(constraint['source_anchor'])
        target = int(constraint['target_anchor'])
        matches = [[old_source, old_target] for old_source, old_target in legacy
                   if abs(source - old_source) <= index_window
                   and abs(target - old_target) <= index_window]
        if matches:
            survivors.append({
                'source_anchor': source,
                'target_anchor': target,
                'matching_legacy_edges': matches,
            })
    return survivors


def chunk_metadata_manifest(chunks: list[Path]) -> dict[str, Any]:
    entries = [(path.name, path.stat().st_size) for path in chunks]
    return {
        'count': len(entries),
        'total_bytes': int(sum(size for _, size in entries)),
        'name_size_manifest_sha256': payload_sha256(entries),
    }


def audit_sequence(*, contract_path: Path, voxel_dir: Path, sequence_id: str,
                   repetition: int, output: Path, protected_map: Path | None,
                   legacy_loop_edges: Path | None) -> dict[str, Any]:
    started = time.monotonic()
    contract, contract_sha = load_contract(contract_path)
    memory = MemoryGuard(float(contract['memory']['maximum_rss_mib']))
    state_path = voxel_dir / 'alidarState.txt'
    report: dict[str, Any] = {
        'schema_version': 1,
        'audit': 'v42_streaming_pose_graph_feasibility',
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'status': 'FAIL',
    }
    try:
        if not state_path.is_file():
            raise FileNotFoundError(state_path)
        state_sha_before = sha256_file(state_path)
        map_sha_before = sha256_file(protected_map) if protected_map else None
        stamps, transforms = load_states(state_path)
        chunks = indexed_chunks(voxel_dir, len(stamps))
        anchors = select_anchor_indices(
            transforms, float(contract['anchor']['minimum_translation_m']))
        if len(anchors) < 2:
            raise ContractError('fewer than two streaming anchors')
        loader = StreamingSubmapLoader(
            transforms, anchors, chunks, contract['submap'], memory)
        descriptors = build_descriptor_index(loader, contract['descriptor'])
        records, qualified = generate_candidates(
            transforms, anchors, descriptors, contract)
        maximum_geometry = int(contract['candidate']['maximum_geometry_verifications'])
        if len(qualified) > maximum_geometry:
            raise MemoryBudgetError(
                f'{len(qualified)} geometry verifications exceed fixed limit {maximum_geometry}')
        for candidate in qualified:
            geometry = register_candidate(candidate, loader, contract)
            candidate['geometry'] = geometry
            candidate['disposition'] = (
                'ACCEPT_RAW_GEOMETRY' if geometry['passed'] else 'REJECT_GEOMETRY')
        constraints = deduplicate_constraints(
            qualified, int(contract['deduplication']['index_window']))
        odometry_edges = [(index, index + 1) for index in range(len(anchors) - 1)]
        loop_edges = [(int(item['source_anchor']), int(item['target_anchor']))
                      for item in constraints]
        if not graph_connected(len(anchors), [*odometry_edges, *loop_edges]):
            raise ContractError('odometry plus loop graph is disconnected')
        legacy = load_legacy_edges(legacy_loop_edges)
        survivors = legacy_survivors(
            constraints, legacy, int(contract['deduplication']['index_window']))
        state_sha_after = sha256_file(state_path)
        map_sha_after = sha256_file(protected_map) if protected_map else None
        if state_sha_before != state_sha_after or map_sha_before != map_sha_after:
            raise ContractError('protected v17 input changed during report-only audit')
        deterministic = {
            'sequence_id': sequence_id,
            'contract_id': contract['contract_id'],
            'contract_sha256': contract_sha,
            'input': {
                'state_sha256': state_sha_before,
                'state_count': int(len(stamps)),
                'chunk_manifest': chunk_metadata_manifest(chunks),
                'protected_map_sha256': map_sha_before,
            },
            'anchor': {
                'count': len(anchors),
                'state_indices_sha256': payload_sha256(anchors),
                'first_stamp': float(stamps[anchors[0]]),
                'last_stamp': float(stamps[anchors[-1]]),
            },
            'submap': {
                'minimum_points': min(descriptors.point_counts),
                'maximum_points': max(descriptors.point_counts),
                'median_points': float(np.median(descriptors.point_counts)),
                'peak_cache_bytes': loader.peak_cache_bytes,
                'peak_submap_bytes': loader.peak_submap_bytes,
            },
            'candidate': {
                'episode_query_count': len(records),
                'descriptor_qualified_count': len(qualified),
                'geometry_pass_count_before_dedup': sum(
                    bool(item.get('geometry', {}).get('passed')) for item in qualified),
                'records': records,
            },
            'constraints': constraints,
            'legacy_challenge': {
                'edge_count': len(legacy),
                'edges_sha256': payload_sha256(legacy),
                'survivors': survivors,
            },
            'graph': {
                'node_count': len(anchors),
                'odometry_edge_count': len(odometry_edges),
                'verified_loop_edge_count': len(constraints),
                'connected': True,
            },
            'protected_inputs_unchanged': True,
        }
        report['deterministic'] = deterministic
        report['deterministic_payload_sha256'] = payload_sha256(deterministic)
        report['status'] = 'PASS'
    except Exception as error:  # retain a fail-closed report for every audit attempt
        report['error_type'] = type(error).__name__
        report['error'] = str(error)
    finally:
        try:
            memory.check('audit_finalize')
        except MemoryBudgetError as error:
            report['status'] = 'FAIL'
            report['error_type'] = type(error).__name__
            report['error'] = str(error)
        report['runtime'] = {
            'wall_seconds': time.monotonic() - started,
            'peak_rss_mib': memory.peak_rss_mib,
            'maximum_rss_mib': memory.maximum_rss_mib,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n',
                          encoding='utf-8')
    return report


def aggregate_reports(*, contract_path: Path, reports: list[Path],
                      expected_sequences: list[str], output: Path,
                      markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_sha = load_contract(contract_path)
    loaded = [(path, json.loads(path.read_text(encoding='utf-8'))) for path in reports]
    groups: dict[str, list[tuple[Path, dict[str, Any]]]] = {
        sequence: [] for sequence in expected_sequences}
    for path, report in loaded:
        sequence = report.get('sequence_id')
        if sequence not in groups:
            raise ContractError(f'unexpected sequence {sequence!r} in {path}')
        groups[sequence].append((path, report))
    required = int(contract['decision']['required_repetitions'])
    sequence_results = []
    complete = True
    repeatable = True
    total_constraints = 0
    total_survivors = 0
    applicable_failures = []
    for sequence in expected_sequences:
        items = sorted(groups[sequence], key=lambda item: int(item[1].get('repetition', 0)))
        statuses = [item[1].get('status') for item in items]
        hashes = [item[1].get('deterministic_payload_sha256') for item in items]
        integrity = [
            item[1].get('contract_sha256') == contract_sha
            and item[1].get('deterministic_payload_sha256') ==
            payload_sha256(item[1].get('deterministic', {}))
            for item in items
        ]
        sequence_complete = (
            len(items) == required
            and all(status == 'PASS' for status in statuses)
            and all(integrity)
        )
        sequence_repeatable = sequence_complete and len(set(hashes)) == 1
        complete &= sequence_complete
        repeatable &= sequence_repeatable
        first = items[0][1].get('deterministic', {}) if items else {}
        candidate = first.get('candidate', {})
        challenge = first.get('legacy_challenge', {})
        constraints = first.get('constraints', [])
        survivors = challenge.get('survivors', [])
        applicable = int(candidate.get('descriptor_qualified_count', 0)) > 0
        new_constraints = max(0, len(constraints) - len(survivors))
        total_constraints += new_constraints
        total_survivors += len(survivors)
        if (contract['decision']['require_new_constraint_per_applicable_sequence']
                and applicable and new_constraints == 0):
            applicable_failures.append(sequence)
        sequence_results.append({
            'sequence_id': sequence,
            'report_count': len(items),
            'complete': sequence_complete,
            'repeatable': sequence_repeatable,
            'deterministic_payload_sha256': hashes[0] if sequence_repeatable else None,
            'episode_query_count': int(candidate.get('episode_query_count', 0)),
            'descriptor_qualified_count': int(candidate.get('descriptor_qualified_count', 0)),
            'verified_constraint_count': len(constraints),
            'legacy_challenge_survivor_count': len(survivors),
            'new_verified_constraint_count': new_constraints,
            'applicable': applicable,
        })
    maximum_survivors = int(contract['decision']['maximum_legacy_challenge_survivors'])
    minimum_constraints = int(contract['decision']['minimum_total_new_verified_constraints'])
    if not complete or not repeatable:
        decision = 'REJECT_V42_INCOMPLETE_OR_NONREPEATABLE_AUDIT'
    elif total_survivors > maximum_survivors:
        decision = 'REJECT_V42_LEGACY_AMBIGUITY_SURVIVED'
    elif total_constraints < minimum_constraints or applicable_failures:
        decision = 'REJECT_V42_NO_COMPLETE_UNAMBIGUOUS_CONSTRAINT_SET'
    else:
        decision = 'AUTHORIZE_V42_EXTERNAL_SPARSE_POSE_GRAPH_IMPLEMENTATION'
    aggregate = {
        'schema_version': 1,
        'audit': 'v42_streaming_pose_graph_feasibility_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'status': 'PASS' if complete and repeatable else 'FAIL',
        'decision': decision,
        'external_sparse_pose_graph_authorized': decision.startswith('AUTHORIZE_'),
        'sequence_results': sequence_results,
        'total_new_verified_constraints': total_constraints,
        'total_legacy_challenge_survivors': total_survivors,
        'applicable_sequences_without_new_constraint': applicable_failures,
        'source_reports': [
            {'path': str(path.resolve()), 'sha256': sha256_file(path)} for path, _ in loaded],
    }
    aggregate['aggregate_payload_sha256'] = payload_sha256({
        key: value for key, value in aggregate.items() if key != 'source_reports'})
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if markdown_output:
        lines = [
            '# v42 streaming external pose-graph feasibility', '',
            f"- decision: `{decision}`",
            f"- external sparse pose graph authorized: `{str(aggregate['external_sparse_pose_graph_authorized']).lower()}`",
            f"- new verified constraints: `{total_constraints}`",
            f"- legacy challenge survivors: `{total_survivors}`", '',
            '## Sequences', '',
        ]
        for item in sequence_results:
            lines.append(
                f"- `{item['sequence_id']}`: repeatable={str(item['repeatable']).lower()}, "
                f"qualified={item['descriptor_qualified_count']}, "
                f"verified={item['verified_constraint_count']}, "
                f"new={item['new_verified_constraint_count']}, "
                f"legacy_survivors={item['legacy_challenge_survivor_count']}")
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return aggregate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    audit = subparsers.add_parser('audit')
    audit.add_argument('--contract', type=Path, required=True)
    audit.add_argument('--voxel-dir', type=Path, required=True)
    audit.add_argument('--sequence-id', required=True)
    audit.add_argument('--repetition', type=int, required=True)
    audit.add_argument('--output', type=Path, required=True)
    audit.add_argument('--protected-map', type=Path)
    audit.add_argument('--legacy-loop-edges', type=Path)
    aggregate = subparsers.add_parser('aggregate')
    aggregate.add_argument('--contract', type=Path, required=True)
    aggregate.add_argument('--report', type=Path, action='append', required=True)
    aggregate.add_argument('--expected-sequence', action='append', required=True)
    aggregate.add_argument('--output', type=Path, required=True)
    aggregate.add_argument('--markdown-output', type=Path)
    return parser.parse_args()


def main() -> int:
    options = parse_args()
    if options.command == 'audit':
        report = audit_sequence(
            contract_path=options.contract,
            voxel_dir=options.voxel_dir,
            sequence_id=options.sequence_id,
            repetition=options.repetition,
            output=options.output,
            protected_map=options.protected_map,
            legacy_loop_edges=options.legacy_loop_edges,
        )
        return 0 if report['status'] == 'PASS' else 2
    aggregate = aggregate_reports(
        contract_path=options.contract,
        reports=options.report,
        expected_sequences=options.expected_sequence,
        output=options.output,
        markdown_output=options.markdown_output,
    )
    return 0 if aggregate['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
