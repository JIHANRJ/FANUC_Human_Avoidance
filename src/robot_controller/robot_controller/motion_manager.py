#!/usr/bin/env python3

import rclpy
import time
import copy
import math
from rclpy.node import Node
from robot_controller.planner import Planner
from geometry_msgs.msg import PoseStamped
from robot_controller.robot_state import RobotState
from robot_controller.constants import MAX_PLANNING_RATE
from robot_controller.planning_scene import PlanningSceneManager
from geometry_msgs.msg import PoseArray
from robot_controller.movegroup_client import MoveGroupClient


class MotionManager(Node):

    def __init__(self):

        super().__init__("motion_manager")
        self.robot_state = RobotState(self)

        self.get_logger().info("")
        self.get_logger().info("==============================")
        self.get_logger().info(" Motion Manager Started ")
        self.get_logger().info("==============================")

        #
        # Latest avoidance target
        #
        self.current_target = None
        self.planner = Planner()

        self.target_updated = False

        #
        # Connect to MoveGroup
        #
        self.move_group = MoveGroupClient(self)
        #
        # Planning Scene
        #
        self.scene = PlanningSceneManager(self)

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
        # Human Pose Subscriber
        #

        self.pose_sub = self.create_subscription(
            PoseArray,
            "/human_pose",
            self.pose_callback,
            10,
        )

        self.human_pose = None

        #
        # Planning timer
        #
        self.timer = self.create_timer(
            0.2,
            self.timer_callback,
        )

        self.get_logger().info("Waiting for robot state...")
        self.last_plan_time = 0.0

        #
        # Hold-pose state
        #



        #
        # Test collision object
        #

        #
        # Nominal pose
        #

        self.nominal_pose = None

        #
        # Goal deadband (meters)
        #

        self.goal_update_threshold = 0.0025      # 3 cm
        self.last_sent_goal = None
    ########################################################

    def target_callback(self, msg):

        self.current_target = msg
        self.target_updated = True

    ########################################################

    def timer_callback(self):

        if self.current_target is None:
            return

        if not self.target_updated:
            return

        current_time = time.time()

        if current_time - self.last_plan_time < (1.0 / MAX_PLANNING_RATE):
            return
        #
        # Update human collision sphere
        #

        if self.human_pose is not None and len(self.human_pose.poses) > 12:

            left = self.human_pose.poses[11].position
            right = self.human_pose.poses[12].position

            torso_x = (left.x + right.x) / 2.0
            torso_y = (left.y + right.y) / 2.0
            torso_z = (left.z + right.z) / 2.0

            self.scene.update_sphere(
                object_id="torso",
                x=torso_x,
                y=torso_y,
                z=torso_z,
                radius=0.20,
                frame="camera_link",
            )

        #TODO: Pass current robot pose to planner
        current_pose = self.robot_state.get_tcp_pose()

        if current_pose is None:
            return
        
        if self.nominal_pose is None:
            self.nominal_pose = copy.deepcopy(current_pose)
            self.get_logger().info("Stored nominal pose.")
        
##
        goal = self.planner.compute_goal(
            current_pose,
            self.current_target,
        )
        if goal is None:
            self.get_logger().warn("Planner returned None")
            return

        self.get_logger().info("Planner produced a goal.")
        #
        # Ignore tiny goal changes
        #

        if goal is not None:

            p = goal.pose.position

            if self.last_sent_goal is not None:

                lp = self.last_sent_goal.pose.position

                dx = p.x - lp.x
                dy = p.y - lp.y
                dz = p.z - lp.z

                distance = math.sqrt(
                    dx * dx +
                    dy * dy +
                    dz * dz
                )

                if distance < self.goal_update_threshold:

                    self.get_logger().info(
                        f"Skipping goal ({distance:.3f} m < {self.goal_update_threshold:.3f} m)"
                    )

                    return
            self.last_sent_goal = copy.deepcopy(goal)
            self.get_logger().info("Goal passed deadband.")

        if goal is None:
            return



        #
        # Only react to NEW targets
        #

        self.target_updated = False


        p = goal.pose.position

        self.get_logger().info("")
        self.get_logger().info("===== SAFE GOAL =====")
        self.get_logger().info(f"X : {p.x:.3f}")
        self.get_logger().info(f"Y : {p.y:.3f}")
        self.get_logger().info(f"Z : {p.z:.3f}")

        #

        success = self.move_group.plan(goal)

        if success:
            self.last_plan_time = current_time
            self.get_logger().info("Planner accepted target.")
        else:
            self.get_logger().warn("Planner rejected target.")
        #
    def pose_callback(self, msg):

        self.human_pose = msg


def main():

    rclpy.init()

    node = MotionManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()