#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class SkeletonPublisher(Node):

    def __init__(self):
        super().__init__("skeleton_publisher")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            "/human_markers",
            qos,
        )

        self.publish_count = 0
        self.last_debug_time = self.get_clock().now()

    def publish_points(self, pose_3d, connections=None):
        markers = MarkerArray()

        valid_count = 0
        line_count = 0

        for idx, point in enumerate(pose_3d):
            marker = Marker()

            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human"
            marker.id = idx

            if np.isnan(point).any():
                marker.action = Marker.DELETE
                markers.markers.append(marker)
                continue

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            markers.markers.append(marker)
            valid_count += 1

        line_marker = Marker()
        line_marker.header.frame_id = "camera_link"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "human_skeleton"
        line_marker.id = 0

        if connections is None:
            connections = []

        for start_idx, end_idx in connections:
            start = pose_3d[start_idx]
            end = pose_3d[end_idx]

            if np.isnan(start).any() or np.isnan(end).any():
                continue

            start_point = Point()
            start_point.x = float(start[0])
            start_point.y = float(start[1])
            start_point.z = float(start[2])

            end_point = Point()
            end_point.x = float(end[0])
            end_point.y = float(end[1])
            end_point.z = float(end[2])

            line_marker.points.append(start_point)
            line_marker.points.append(end_point)
            line_count += 1

        if line_count > 0:
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.025
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0
        else:
            line_marker.action = Marker.DELETE

        markers.markers.append(line_marker)

        self.publisher.publish(markers)
        self.publish_count += 1

        now = self.get_clock().now()
        elapsed = (now - self.last_debug_time).nanoseconds / 1e9

        if elapsed >= 1.0:
            hz = self.publish_count / elapsed
            self.get_logger().info(
                f"published MarkerArray: markers={len(markers.markers)} "
                f"valid_points={valid_count} lines={line_count} "
                f"rate={hz:.1f} Hz"
            )
            self.publish_count = 0
            self.last_debug_time = now

        return valid_count, line_count


def main():
    rclpy.init()

    node = SkeletonPublisher()

    try:
        test_pose = np.full((33, 3), math.nan, dtype=np.float32)
        node.publish_points(test_pose)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
