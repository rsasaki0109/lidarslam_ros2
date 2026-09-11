#!/usr/bin/env python3
"""Replay validated scalar shadow observations on an opt-in ROS 1 topic.

This is a candidate-screen bridge, not a camera estimator.  It publishes the
already validated report payload as ``geometry_msgs/Vector3Stamped``:
``vector.x`` is signed m/s, ``vector.y`` is normalized confidence, and the
header timestamp is the source timestamp in seconds.  The bridge waits for
ROS simulated time so that the receiver's stale check remains meaningful.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def _finite(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{field} must be finite')
    return result


def validated_observations(
        report: dict[str, Any],) -> list[dict[str, float]]:
    """Validate the producer report before any ROS import or publication."""
    if report.get('accuracy_ground_truth_accessed') is not False:
        raise ValueError('shadow report is not ground-truth-free')
    if report.get('decision') != 'GO_REPORT_ONLY_SHADOW_SOURCE':
        raise ValueError(f"shadow report did not pass: {report.get('decision')}")
    contract = report.get('contract')
    if not isinstance(contract, dict):
        raise ValueError('shadow report has no contract')
    max_speed = _finite(contract.get('max_speed_mps'), 'max_speed_mps')
    observations = report.get('observations')
    if not isinstance(observations, list):
        raise ValueError('shadow report has no observations')
    validated: list[dict[str, float]] = []
    previous_stamp = -math.inf
    for observation in observations:
        if not isinstance(observation, dict):
            raise ValueError('shadow observation is not an object')
        stamp = _finite(observation.get('stamp_sec'), 'stamp_sec')
        velocity = _finite(observation.get('velocity_mps'), 'velocity_mps')
        confidence = _finite(observation.get('confidence'), 'confidence')
        if stamp < previous_stamp:
            raise ValueError('shadow observation timestamps are not ordered')
        if abs(velocity) > max_speed:
            raise ValueError('shadow observation exceeds speed bound')
        if not 0.0 <= confidence <= 1.0:
            raise ValueError('shadow observation confidence is outside [0,1]')
        validated.append({
            'stamp_sec': stamp,
            'velocity_mps': velocity,
            'confidence': confidence,
        })
        previous_stamp = stamp
    return validated


def load_report(path: Path) -> dict[str, Any]:
    """Load the JSON report without opening any reference artifact."""
    return json.loads(path.read_text(encoding='utf-8'))


def publish_report(
        report_path: Path, topic: str, max_lateness_sec: float,
        ) -> dict[str, int]:
    """Publish observations against ROS simulated time and count outcomes."""
    if max_lateness_sec <= 0.0:
        raise ValueError('max lateness must be positive')
    report = load_report(report_path)
    observations = validated_observations(report)

    import rospy
    from geometry_msgs.msg import Vector3Stamped

    rospy.init_node('visual_weak_axis_shadow_replay', anonymous=False)
    publisher = rospy.Publisher(topic, Vector3Stamped, queue_size=100)
    published = 0
    skipped_late = 0
    for observation in observations:
        stamp = observation['stamp_sec']
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < stamp:
            rospy.sleep(0.001)
        if rospy.is_shutdown():
            break
        lateness = rospy.Time.now().to_sec() - stamp
        if lateness > max_lateness_sec:
            skipped_late += 1
            continue
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.vector.x = observation['velocity_mps']
        message.vector.y = observation['confidence']
        message.vector.z = 0.0
        publisher.publish(message)
        published += 1
    return {'validated': len(observations), 'published': published,
            'skipped_late': skipped_late}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument(
        '--topic', default='/voxel_slam/visual_longitudinal_shadow')
    parser.add_argument('--max-lateness-sec', type=float, default=0.15)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = publish_report(args.report, args.topic, args.max_lateness_sec)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
