from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener

import rclpy


class RobotState:

    def __init__(self, node):

        self.node = node

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            node,
        )

    ########################################################

    def get_tcp_transform(
        self,
        world="world",
        tcp="flange",
    ):

        try:

            return self.tf_buffer.lookup_transform(
                world,
                tcp,
                rclpy.time.Time(),
            )

        except Exception:

            return None

    ########################################################

    def get_tcp_pose(self):

        transform = self.get_tcp_transform()

        if transform is None:
            return None

        pose = PoseStamped()

        pose.header = transform.header

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z

        pose.pose.orientation = transform.transform.rotation

        return pose