#!/usr/bin/env python3
"""Shared MID-360 robot preflight and run-planning helpers."""

from __future__ import annotations

from lidarslam_benchmark_tools.lidarslam_tools.serialization import payload_to_json
from lidarslam_benchmark_tools.lidarslam_tools.mid360_profile import (
    RobotProfileLoader,
    render_robot_profile_report,
    resolve_robot_frames,
)
from lidarslam_benchmark_tools.lidarslam_tools.mid360_planning import (
    Mid360MapRunPlanner,
    Mid360RunDiagnosisPlanner,
)
from lidarslam_benchmark_tools.lidarslam_tools.mid360_reports import Mid360ReadinessReporter, Mid360RunManifestWriter
from lidarslam_benchmark_tools.lidarslam_tools.autoware_preflight import AutowarePreflightAdapter
from lidarslam_benchmark_tools.lidarslam_tools.mid360_bag_diagnostics import Mid360BagDiagnosticsBuilder
from lidarslam_benchmark_tools.lidarslam_tools.mid360_preflight import MID360_PROFILE_ID, Mid360RobotPreflight
from lidarslam_benchmark_tools.lidarslam_tools.mid360_models import (
    DiagnosisPlan,
    MapRunOptions,
    MapRunPlan,
    MessageSample,
    PreflightCheck,
    RobotFrames,
    RobotProfile,
    TopicSelection,
)


__all__ = [
    'AutowarePreflightAdapter', 'DiagnosisPlan', 'MID360_PROFILE_ID',
    'MapRunOptions', 'MapRunPlan', 'MessageSample', 'Mid360BagDiagnosticsBuilder',
    'Mid360MapRunPlanner', 'Mid360ReadinessReporter', 'Mid360RobotPreflight',
    'Mid360RunDiagnosisPlanner', 'Mid360RunManifestWriter', 'PreflightCheck',
    'RobotFrames', 'RobotProfile', 'RobotProfileLoader', 'TopicSelection',
    'payload_to_json', 'render_robot_profile_report', 'resolve_robot_frames',
]


if __name__ == "__main__":
    raise SystemExit("mid360_robot_tools is a library module; import it instead of running it directly.")
