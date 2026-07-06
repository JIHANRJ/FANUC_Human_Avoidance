#!/usr/bin/env python3

import time
from geometry_msgs.msg import PoseStamped


class NominalMotion:

    def __init__(self):

        self.start_time = time.time()

    ########################################################

    def hold_pose(self):

        goal = PoseStamped()

        goal.header.frame_id = "world"

        goal.pose.position.x = 0.70
        goal.pose.position.y = -0.15
        goal.pose.position.z = 0.95

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        return goal