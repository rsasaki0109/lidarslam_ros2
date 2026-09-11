#!/usr/bin/env python3
"""Subscribe to nav_msgs/Path and save the latest path as TUM format."""

import argparse
import sys


def _require_ros2():
    """Return ROS2 symbols or a precise execution-time dependency error."""
    try:
        import rclpy
        from nav_msgs.msg import Path
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.parameter import Parameter
    except ImportError as error:
        raise RuntimeError(
            'path_to_tum requires ROS2 rclpy and nav_msgs at execution time'
        ) from error
    return rclpy, Node, Parameter, ExternalShutdownException, Path


def _build_node_class(Node, Parameter, Path):
    class PathToTum(Node):
        """ROS2 node implementation, built only after dependencies load."""

        def __init__(self, topic, output, use_sim_time):
            super().__init__('path_to_tum')
            if use_sim_time:
                self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.output = output
            self.sub = self.create_subscription(Path, topic, self.cb, 10)
            self.get_logger().info(f'Subscribed to {topic}, will save to {output}')

        def cb(self, msg):
            self.get_logger().info(f'Received path with {len(msg.poses)} poses')
            with open(self.output, 'w') as f:
                for ps in msg.poses:
                    t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9
                    p = ps.pose.position
                    q = ps.pose.orientation
                    f.write(f'{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')

    return PathToTum


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/modified_path')
    ap.add_argument('--output', required=True)
    ap.add_argument('--use-sim-time', default='true')
    args = ap.parse_args()

    rclpy, Node, Parameter, ExternalShutdownException, Path = _require_ros2()
    PathToTum = _build_node_class(Node, Parameter, Path)

    # Let rclpy install its own SIGINT/SIGTERM handlers (signal_handler_options
    # defaults to ALL). Custom signal.signal() handlers do not interrupt
    # rclpy.spin() because rcl_wait blocks in C and never yields to Python.
    rclpy.init(args=sys.argv)
    node = PathToTum(args.topic, args.output,
                     args.use_sim_time.lower() in ('true', '1', 'yes'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
