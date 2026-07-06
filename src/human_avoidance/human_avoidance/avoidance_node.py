#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


class AvoidanceNode(Node):

    def __init__(self):

        #
        # Filtered avoidance target
        #

        self.filtered_target = None

        #
        # Low-pass filter coefficient
        #

        self.alpha = 0.2

        super().__init__("avoidance_node")

        self.get_logger().info("=================================")
        self.get_logger().info(" Human Avoidance Node Started ")
        self.get_logger().info("=================================")

        # -------------------------------
        # TF
        # -------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # -------------------------------
        # Human pose subscriber
        # -------------------------------

        self.latest_pose = None

        self.pose_sub = self.create_subscription(
            PoseArray,
            "/human_pose",
            self.pose_callback,
            10,
        )

        # -------------------------------
        # Timer
        # -------------------------------

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/avoidance_markers",
            10
        )
        # Avoidance parameters
        self.safe_distance = 0.60   # meters
        self.avoid_offset = 0.20    # meters

        # Target 

        self.target_pub = self.create_publisher(
            PoseStamped,
            "/avoidance_target",
            10,
        )

 
    # --------------------------------------------------------

    def pose_callback(self, msg):

        self.latest_pose = msg

    # --------------------------------------------------------

    def timer_callback(self):

        if self.latest_pose is None:
            self.get_logger().info("Waiting for human pose...")
            return

        try:

            tf = self.tf_buffer.lookup_transform(
                "world",
                "end_effector",
                rclpy.time.Time(),
            )

            tcp = tf.transform.translation

            tcp_np = np.array([
                tcp.x,
                tcp.y,
                tcp.z
            ])

            closest_distance = float("inf")
            closest_index = -1
            closest_point = None

            if len(self.latest_pose.poses) == 0:
                self.get_logger().warn("No human landmarks.")
                return

            for i, pose in enumerate(self.latest_pose.poses):

                p_camera = np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ])

                p_world = self.camera_to_world(p_camera)

                d = np.linalg.norm(tcp_np - p_world)

                if d < closest_distance:
                    closest_distance = d
                    closest_index = i
                    closest_point = p_world


            print("\n===========================")
            print("Robot TCP")
            print(f"X : {tcp.x:.3f}")
            print(f"Y : {tcp.y:.3f}")
            print(f"Z : {tcp.z:.3f}")

            print()

            print(f"Human landmarks : {len(self.latest_pose.poses)}")
            print(f"Closest joint   : {closest_index}")
            print("Closest point (world)")
            print(closest_point)
            print("TCP (world)")
            print(tcp_np)
            print(f"Distance        : {closest_distance:.3f} m")

            if closest_point is None:
                self.get_logger().warn("No valid human detected.")
                return


            # --------------------------------------------------
            # Compute avoidance target
            # --------------------------------------------------


            if closest_distance < self.safe_distance:

                direction = tcp_np - closest_point

                norm = np.linalg.norm(direction)

                if norm > 1e-6:
                    direction /= norm
                else:
                    direction = np.zeros(3)

                target = tcp_np + self.avoid_offset*direction

                print("AVOIDING HUMAN")

            else:

                target = tcp_np

                print("Human is far away")

            print("Target")
            print(target)

            print("Repulsion direction")

            #
            # Low-pass filter
            #

            if self.filtered_target is None:

                self.filtered_target = target.copy()

            else:

                self.filtered_target = (
                    (1.0 - self.alpha) * self.filtered_target
                    + self.alpha * target
                )

            target = self.filtered_target
            # print(direction) # debugging

            markers = MarkerArray()
 
            markers.markers.append( # Blue sphere, marking the origin
                self.make_sphere(
                    marker_id=0,
                    frame_id="camera_link",
                    xyz=[0.0, 0.0, 0.0],
                    r=0.0,
                    g=0.0,
                    b=1.0,
                    scale=0.06,
                )
            )

            markers.markers.append( # Green sphere, marking the TCP
                self.make_sphere(
                    marker_id=1,
                    frame_id="world",
                    xyz=tcp_np,
                    r=0.0,
                    g=1.0,
                    b=0.0,
                    scale=0.06,
                )
            )

            markers.markers.append( # yellow sphere, marking the closest point
                self.make_sphere(
                    marker_id=2,
                    frame_id="world",
                    xyz=closest_point,
                    r=1.0,
                    g=1.0,
                    b=0.0,
                    scale=0.06,
                )
            )

            markers.markers.append( # Purple sphere, marking the target
                self.make_sphere(
                    marker_id=3,
                    frame_id="world",
                    xyz=target,
                    r=1.0,
                    g=0.0,
                    b=1.0,
                    scale=0.06,
                )
            )
            self.marker_pub.publish(markers)

            msg = PoseStamped()

            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.position.x = float(target[0])
            msg.pose.position.y = float(target[1])
            msg.pose.position.z = float(target[2])

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            self.target_pub.publish(msg)


        except TransformException as e:

            self.get_logger().warn(str(e))

    def make_sphere(self, marker_id, frame_id, xyz, r, g, b, scale=0.05):

        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "avoidance"
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        return marker
    
    def camera_to_world(self, point_camera):
        """
        Convert a 3D point from camera_link into world coordinates.
        """

        p = PointStamped()
        p.header.frame_id = "camera_link"
        p.header.stamp = rclpy.time.Time().to_msg()

        p.point.x = float(point_camera[0])
        p.point.y = float(point_camera[1])
        p.point.z = float(point_camera[2])

        transform = self.tf_buffer.lookup_transform(
            "world",
            "camera_link",
            rclpy.time.Time()
        )

        p_world = do_transform_point(p, transform)

        return np.array([
            p_world.point.x,
            p_world.point.y,
            p_world.point.z
        ])


def main(args=None):

    rclpy.init(args=args)

    node = AvoidanceNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()