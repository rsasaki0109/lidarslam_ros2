#!/usr/bin/env python3
"""Single source of truth for maintained product workflow profiles."""

from __future__ import annotations

from typing import Any, NamedTuple


class MaintainedProfile(NamedTuple):
    """Public identity and help text for one executable product profile."""

    profile_id: str
    description: str


MAINTAINED_PROFILES = (
    MaintainedProfile(
        'rko_lio_graph_public_path',
        'PointCloud2 + Imu through RKO-LIO and graph_based_slam.',
    ),
    MaintainedProfile(
        'rko_lio_graph_mid360_preset',
        'Livox/MID360 PointCloud2 + Imu with tracked tuned params.',
    ),
    MaintainedProfile(
        'pointcloud_gnss_smoke',
        'PointCloud2 + NavSatFix smoke workflow.',
    ),
    MaintainedProfile(
        'packet_applanix_smoke',
        'VelodyneScan + Applanix GSOF49 smoke workflow.',
    ),
)

PROFILE_IDS = tuple(profile.profile_id for profile in MAINTAINED_PROFILES)
PROFILE_HELP = tuple(
    (profile.profile_id, profile.description)
    for profile in MAINTAINED_PROFILES
)


def select_profile(
    payload: dict[str, Any],
    forced_profile_id: str | None = None,
) -> str:
    """Select the same maintained profile for every human and batch path."""
    if forced_profile_id:
        return forced_profile_id

    recommendations = payload['recommendations']
    recommendation_ids = {item['id'] for item in recommendations}
    summary = payload['summary']
    pointcloud_topics = summary['topics']['pointcloud2']
    imu_topics = summary['topics']['imu']
    bag_path_lower = summary['bag_path'].lower()
    looks_like_livox = (
        'mid360' in bag_path_lower
        or any(
            'livox' in item['name'].lower()
            for item in pointcloud_topics + imu_topics
        )
    )
    if looks_like_livox and 'rko_lio_graph_mid360_preset' in recommendation_ids:
        return 'rko_lio_graph_mid360_preset'

    recommended_profile_id = payload['recommended_profile_id']
    if not recommended_profile_id:
        raise RuntimeError('no compatible public path was found for this bag')
    return recommended_profile_id

if len(PROFILE_IDS) != len(set(PROFILE_IDS)):
    raise RuntimeError('maintained product profile IDs must be unique')
