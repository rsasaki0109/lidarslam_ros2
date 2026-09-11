#!/usr/bin/env python3
"""Generate a small, readable MID-360-style rosbag2 for local testing."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import shutil
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SampleBagConfig:
    """Configuration for a synthetic MID-360 robot bag."""

    output_path: Path
    duration_sec: float = 5.0
    pointcloud_rate_hz: float = 10.0
    imu_rate_hz: float = 100.0
    point_count: int = 32
    pointcloud_topic: str = '/livox/lidar'
    imu_topic: str = '/livox/imu'
    tf_static_topic: str = '/tf_static'
    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'
    write_tf_static: bool = True
    force: bool = False
    start_offset_sec: float = 0.0


@dataclass(frozen=True)
class SampleBagSummary:
    """Summary of a generated sample bag."""

    output_path: str
    duration_sec: float
    message_count: int
    pointcloud_messages: int
    imu_messages: int
    tf_static_messages: int
    topics: dict[str, str]
    frames: dict[str, str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def import_rosbag_writer_modules():
    """Import rosbags lazily so tests can skip cleanly if it is unavailable."""
    try:
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise ModuleNotFoundError(
            'rosbags is required to generate a MID-360 sample bag',
        ) from exc
    return Writer, Stores, get_typestore


def validate_config(config: SampleBagConfig) -> None:
    """Validate generator options before touching the output path."""
    if config.duration_sec <= 0.0:
        raise ValueError('--duration-sec must be greater than zero')
    if config.pointcloud_rate_hz <= 0.0:
        raise ValueError('--pointcloud-rate-hz must be greater than zero')
    if config.imu_rate_hz <= 0.0:
        raise ValueError('--imu-rate-hz must be greater than zero')
    if config.point_count <= 0:
        raise ValueError('--point-count must be greater than zero')
    for field, value in (
        ('pointcloud_topic', config.pointcloud_topic),
        ('imu_topic', config.imu_topic),
        ('tf_static_topic', config.tf_static_topic),
        ('base_frame', config.base_frame),
        ('lidar_frame', config.lidar_frame),
        ('imu_frame', config.imu_frame),
    ):
        if not value:
            raise ValueError(f'{field} must be non-empty')


def stamp_ns_from_seconds(seconds: float) -> int:
    """Convert seconds to integer nanoseconds."""
    return int(round(seconds * 1_000_000_000))


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    """Split nanoseconds into ROS Time fields."""
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


def regular_stamps(duration_sec: float, rate_hz: float) -> list[int]:
    """Return regular message stamps from zero up to, but not including, duration."""
    period_ns = max(1, int(round(1_000_000_000 / rate_hz)))
    duration_ns = stamp_ns_from_seconds(duration_sec)
    stamps = list(range(0, duration_ns, period_ns))
    return stamps or [0]


class Mid360SampleBagWriter:
    """Write small PointCloud2, Imu, and TFMessage streams into rosbag2."""

    def __init__(self, config: SampleBagConfig) -> None:
        self._config = config
        validate_config(config)

    def write(self) -> SampleBagSummary:
        Writer, Stores, get_typestore = import_rosbag_writer_modules()
        typestore = get_typestore(Stores.LATEST)
        output_path = self._config.output_path.expanduser().resolve()
        self._prepare_output(output_path)

        offset_ns = stamp_ns_from_seconds(self._config.start_offset_sec)
        pointcloud_stamps = [
            stamp + offset_ns
            for stamp in regular_stamps(
                self._config.duration_sec,
                self._config.pointcloud_rate_hz,
            )
        ]
        imu_stamps = [
            stamp + offset_ns
            for stamp in regular_stamps(self._config.duration_sec, self._config.imu_rate_hz)
        ]

        messages: list[tuple[int, str, object, str]] = []
        if self._config.write_tf_static:
            messages.append((
                offset_ns,
                self._config.tf_static_topic,
                self._build_tf_static(typestore, offset_ns),
                'tf2_msgs/msg/TFMessage',
            ))
        for index, stamp_ns in enumerate(pointcloud_stamps):
            messages.append((
                stamp_ns,
                self._config.pointcloud_topic,
                self._build_pointcloud(typestore, stamp_ns, index),
                'sensor_msgs/msg/PointCloud2',
            ))
        for index, stamp_ns in enumerate(imu_stamps):
            messages.append((
                stamp_ns,
                self._config.imu_topic,
                self._build_imu(typestore, stamp_ns, index),
                'sensor_msgs/msg/Imu',
            ))
        messages.sort(key=lambda item: (item[0], self._topic_priority(item[1])))

        with Writer(output_path, version=8) as writer:
            connections = {
                self._config.pointcloud_topic: writer.add_connection(
                    self._config.pointcloud_topic,
                    'sensor_msgs/msg/PointCloud2',
                    typestore=typestore,
                ),
                self._config.imu_topic: writer.add_connection(
                    self._config.imu_topic,
                    'sensor_msgs/msg/Imu',
                    typestore=typestore,
                ),
            }
            if self._config.write_tf_static:
                connections[self._config.tf_static_topic] = writer.add_connection(
                    self._config.tf_static_topic,
                    'tf2_msgs/msg/TFMessage',
                    typestore=typestore,
                )
            for stamp_ns, topic, msg, msg_type in messages:
                writer.write(
                    connections[topic],
                    stamp_ns,
                    typestore.serialize_cdr(msg, msg_type),
                )

        return SampleBagSummary(
            output_path=str(output_path),
            duration_sec=self._config.duration_sec,
            message_count=len(messages),
            pointcloud_messages=len(pointcloud_stamps),
            imu_messages=len(imu_stamps),
            tf_static_messages=1 if self._config.write_tf_static else 0,
            topics={
                'pointcloud': self._config.pointcloud_topic,
                'imu': self._config.imu_topic,
                'tf_static': self._config.tf_static_topic,
            },
            frames={
                'base_frame': self._config.base_frame,
                'lidar_frame': self._config.lidar_frame,
                'imu_frame': self._config.imu_frame,
            },
        )

    def _prepare_output(self, output_path: Path) -> None:
        if output_path.exists():
            if not self._config.force:
                raise FileExistsError(f'output bag exists (use --force): {output_path}')
            if output_path.is_dir():
                shutil.rmtree(output_path)
            else:
                output_path.unlink()
        output_path.parent.mkdir(parents=True, exist_ok=True)

    def _build_header(self, typestore: Any, stamp_ns: int, frame_id: str) -> Any:
        Header = typestore.types['std_msgs/msg/Header']
        Time = typestore.types['builtin_interfaces/msg/Time']
        sec, nanosec = sec_nsec_from_ns(stamp_ns)
        return Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame_id)

    def _build_pointcloud(self, typestore: Any, stamp_ns: int, sequence_index: int) -> Any:
        PointField = typestore.types['sensor_msgs/msg/PointField']
        PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
        point_step = 20
        row_step = point_step * self._config.point_count
        data = np.zeros(row_step, dtype=np.uint8)
        points = data.view(dtype=np.float32).reshape(self._config.point_count, 5)
        phase = sequence_index * 0.1
        for point_index in range(self._config.point_count):
            angle = (2.0 * math.pi * point_index / self._config.point_count) + phase
            radius = 1.5 + 0.02 * (point_index % 7)
            points[point_index, 0] = radius * math.cos(angle)
            points[point_index, 1] = radius * math.sin(angle)
            points[point_index, 2] = 0.2 + 0.01 * (point_index % 11)
            points[point_index, 3] = float((sequence_index + point_index) % 255) / 255.0
            points[point_index, 4] = (
                point_index
                / (self._config.point_count * self._config.pointcloud_rate_hz)
            )

        return PointCloud2(
            header=self._build_header(typestore, stamp_ns, self._config.lidar_frame),
            height=1,
            width=self._config.point_count,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            ],
            is_bigendian=False,
            point_step=point_step,
            row_step=row_step,
            data=data,
            is_dense=True,
        )

    def _build_imu(self, typestore: Any, stamp_ns: int, sequence_index: int) -> Any:
        Imu = typestore.types['sensor_msgs/msg/Imu']
        Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
        Vector3 = typestore.types['geometry_msgs/msg/Vector3']
        covariance = np.zeros(9, dtype=np.float64)
        covariance[0] = -1.0
        yaw_rate = 0.01 * math.sin(sequence_index * 0.02)
        return Imu(
            header=self._build_header(typestore, stamp_ns, self._config.imu_frame),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=covariance,
            angular_velocity=Vector3(x=0.0, y=0.0, z=yaw_rate),
            angular_velocity_covariance=covariance,
            linear_acceleration=Vector3(x=0.0, y=0.0, z=9.80665),
            linear_acceleration_covariance=covariance,
        )

    def _build_tf_static(self, typestore: Any, stamp_ns: int) -> Any:
        TFMessage = typestore.types['tf2_msgs/msg/TFMessage']
        transforms = [
            self._build_transform(
                typestore,
                stamp_ns,
                parent=self._config.base_frame,
                child=self._config.lidar_frame,
                x=0.0,
                y=0.0,
                z=0.25,
            )
        ]
        if self._config.imu_frame != self._config.lidar_frame:
            transforms.append(
                self._build_transform(
                    typestore,
                    stamp_ns,
                    parent=self._config.base_frame,
                    child=self._config.imu_frame,
                    x=0.0,
                    y=0.0,
                    z=0.25,
                )
            )
        return TFMessage(transforms=transforms)

    def _build_transform(
        self,
        typestore: Any,
        stamp_ns: int,
        *,
        parent: str,
        child: str,
        x: float,
        y: float,
        z: float,
    ) -> Any:
        TransformStamped = typestore.types['geometry_msgs/msg/TransformStamped']
        Transform = typestore.types['geometry_msgs/msg/Transform']
        Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
        Vector3 = typestore.types['geometry_msgs/msg/Vector3']
        return TransformStamped(
            header=self._build_header(typestore, stamp_ns, parent),
            child_frame_id=child,
            transform=Transform(
                translation=Vector3(x=x, y=y, z=z),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

    def _topic_priority(self, topic: str) -> int:
        if topic == self._config.tf_static_topic:
            return 0
        if topic == self._config.imu_topic:
            return 1
        return 2


def render_summary(summary: SampleBagSummary) -> str:
    """Render a concise human-readable generation summary."""
    return '\n'.join([
        'MID-360 sample bag generated',
        f'bag: {summary.output_path}',
        f'duration_sec: {summary.duration_sec:.3f}',
        f'pointcloud: {summary.pointcloud_messages} messages on {summary.topics["pointcloud"]}',
        f'imu: {summary.imu_messages} messages on {summary.topics["imu"]}',
        f'tf_static: {summary.tf_static_messages} messages on {summary.topics["tf_static"]}',
        f'base_frame: {summary.frames["base_frame"]}',
        f'lidar_frame: {summary.frames["lidar_frame"]}',
        f'imu_frame: {summary.frames["imu_frame"]}',
    ])


if __name__ == "__main__":
    raise SystemExit("mid360_robot_sample_bag is a library module; import it instead of running it directly.")
