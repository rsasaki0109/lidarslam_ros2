#!/usr/bin/env python3

import argparse
import os
import time


def _require_ros2():
    """Return TF/ROS2 symbols or a precise execution-time error."""
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        import tf2_ros
    except ImportError as error:
        raise RuntimeError(
            'tf_to_tum requires ROS2 rclpy and tf2_ros at execution time'
        ) from error
    return rclpy, Duration, Node, Parameter, tf2_ros


def _parse_bool(s: str) -> bool:
    v = (s or "").strip().lower()
    return v in ("1", "true", "t", "yes", "y", "on")


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Log a TF transform to TUM trajectory format: t x y z qx qy qz qw"
    )
    ap.add_argument("--parent-frame", default="map")
    ap.add_argument("--child-frame", default="base_link")
    ap.add_argument("--output", required=True, help="Output file path (.tum/.txt)")
    ap.add_argument("--rate", type=float, default=50.0, help="Logging rate [Hz]")
    ap.add_argument(
        "--use-sim-time",
        default="true",
        help="Use simulated time (/clock). true|false (default: true)",
    )
    ap.add_argument(
        "--tf-cache",
        type=float,
        default=10.0,
        help="TF buffer cache time [s] (default: 10)",
    )
    ap.add_argument(
        "--warn-interval",
        type=float,
        default=5.0,
        help="If TF is missing, print a warning at this interval [s] (default: 5)",
    )
    args = ap.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0")

    use_sim_time = _parse_bool(args.use_sim_time)

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    rclpy, Duration, Node, Parameter, tf2_ros = _require_ros2()
    rclpy.init()
    node = Node("tf_to_tum")
    node.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))]
    )

    buffer = tf2_ros.Buffer(cache_time=Duration(seconds=float(args.tf_cache)))
    _listener = tf2_ros.TransformListener(buffer, node, spin_thread=True)

    last_stamp = None
    last_warn = 0.0

    try:
        with open(args.output, "w", encoding="utf-8", buffering=1) as f:
            while rclpy.ok():
                try:
                    tf = buffer.lookup_transform(
                        args.parent_frame, args.child_frame, rclpy.time.Time()
                    )
                except Exception as e:  # tf2_ros.TransformException is not always available in py
                    now = time.time()
                    if now - last_warn >= float(args.warn_interval):
                        node.get_logger().warn(
                            f"waiting for TF {args.parent_frame}->{args.child_frame}: {e}"
                        )
                        last_warn = now
                    time.sleep(min(0.1, 1.0 / float(args.rate)))
                    continue

                stamp = tf.header.stamp
                if (
                    last_stamp is not None
                    and stamp.sec == last_stamp.sec
                    and stamp.nanosec == last_stamp.nanosec
                ):
                    time.sleep(1.0 / float(args.rate))
                    continue
                last_stamp = stamp

                ts = _stamp_to_float(stamp)
                t = tf.transform.translation
                q = tf.transform.rotation
                f.write(
                    f"{ts:.9f} {t.x:.6f} {t.y:.6f} {t.z:.6f} "
                    f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
                )
                time.sleep(1.0 / float(args.rate))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
