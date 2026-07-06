from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import OrientationConstraint
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class MoveGroupClient:

    def __init__(self, node):

        self.node = node

        self.client = ActionClient(
            node,
            MoveGroup,
            "/move_action",
        )
        #
        # Goal state
        #

        self.goal_active = False

        self.goal_handle = None

        self.node.get_logger().info(
            "Waiting for MoveGroup Action Server..."
        )

        while not self.client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn(
                "Still waiting for /move_action..."
            )
            time.sleep(0.5)

        self.node.get_logger().info(
            "Connected to MoveGroup!"
        )

    ##########################################################

    ##########################################################

    def create_goal(self, goal_pose):

        goal = MoveGroup.Goal()

        request = goal.request

        request.group_name = "manipulator"

        request.allowed_planning_time = 2.0

        request.num_planning_attempts = 5

        request.max_velocity_scaling_factor = 0.30

        request.max_acceleration_scaling_factor = 0.30

        #
        # Position Constraint
        #

        constraint = Constraints()

        position = PositionConstraint()

        position.header = goal_pose.header

        position.link_name = "flange"

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.SPHERE

        primitive.dimensions = [0.005]

        position.constraint_region.primitives.append(
            primitive
        )

        pose = Pose()

        pose.position = goal_pose.pose.position

        position.constraint_region.primitive_poses.append(
            pose
        )

        position.weight = 1.0

        constraint.position_constraints.append(
            position
        )

        #
        # Orientation Constraint
        #

        orientation = OrientationConstraint()

        orientation.header = goal_pose.header

        orientation.link_name = "flange"

        orientation.orientation = goal_pose.pose.orientation

        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05

        orientation.weight = 1.0

        constraint.orientation_constraints.append(
            orientation
        )
        request.goal_constraints.append(
            constraint
        )

        return goal

    def plan(self, goal_pose):

        if self.goal_active:

            self.node.get_logger().warn(
                "MoveGroup is busy."
            )

            return False

        p = goal_pose.pose.position

        self.node.get_logger().info("")
        self.node.get_logger().info("Planning request received")
        self.node.get_logger().info(f"Target X : {p.x:.3f}")
        self.node.get_logger().info(f"Target Y : {p.y:.3f}")
        self.node.get_logger().info(f"Target Z : {p.z:.3f}")

        try:

            goal = self.create_goal(goal_pose)

            self.node.get_logger().info(
                "MoveGroup goal created."
            )
            self.goal_active = True

            future = self.client.send_goal_async(goal)

            future.add_done_callback(
                self.goal_response_callback
            )



            self.node.get_logger().info(
                "Goal sent to MoveGroup."
            )

            return True

        except Exception as e:

            self.node.get_logger().error(
                f"Failed creating MoveGroup goal: {e}"
            )

            return False


    ##########################################################

    def goal_response_callback(self, future):

        self.node.get_logger().info(
         "Entered goal_response_callback()"
        )

        try:
            self.goal_handle = future.result()

            if not self.goal_handle.accepted:

                self.node.get_logger().warn(
                    "Goal rejected."
                )

                self.goal_active = False

                return

        except Exception as e:

            self.node.get_logger().error(
                f"Goal failed: {e}"
            )

            self.goal_active = False

            return

        self.node.get_logger().info(
            "Goal accepted."
        )

        result_future = self.goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    ##########################################################


    def result_callback(self, future):
        self.node.get_logger().info(
    "Entered result_callback()"
        )

        try:

            result = future.result().result

            self.node.get_logger().info(
                "MoveGroup finished."
            )

        except Exception as e:

            self.node.get_logger().error(
                f"MoveGroup failed: {e}"
            )

        self.goal_active = False