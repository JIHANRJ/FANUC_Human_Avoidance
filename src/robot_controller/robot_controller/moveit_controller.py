#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy


class MoveItController(Node):

    def __init__(self):
        super().__init__("moveit_controller")

        self.get_logger().info("")
        self.get_logger().info("==============================")
        self.get_logger().info(" MoveIt Controller Started ")
        self.get_logger().info("==============================")

        # Latest avoidance target
        self.current_target = None

        #
        # Initialise MoveItPy
        #
        self.moveit = MoveItPy(node_name="moveit_py")

        self.planning_component = self.moveit.get_planning_component(
            "manipulator"
        )

        self.get_logger().info("MoveIt initialized successfully")
        self.get_logger().info(
            f"Planning group : {self.planning_component.planning_group_name}"
        )

        #
        # Subscriber
        #
        self.subscription = self.create_subscription(
            PoseStamped,
            "/avoidance_target",
            self.target_callback,
            10,
        )

        #
        # Timer
        #
        self.timer = self.create_timer(
            0.2,
            self.timer_callback,
        )

    def target_callback(self, msg):

        self.current_target = msg

    def timer_callback(self):

        if self.current_target is None:
            self.get_logger().info(
                "Waiting for avoidance target..."
            )
            return

        p = self.current_target.pose.position

        self.get_logger().info("")
        self.get_logger().info("Received avoidance target")
        self.get_logger().info(f"X : {p.x:.3f}")
        self.get_logger().info(f"Y : {p.y:.3f}")
        self.get_logger().info(f"Z : {p.z:.3f}")


def main():

    rclpy.init()

    node = MoveItController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()