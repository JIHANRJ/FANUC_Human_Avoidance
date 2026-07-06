"""
planner.py

Receives an avoidance target and produces a safe goal pose.

This module contains NO ROS publishers,
NO subscribers,
NO MoveIt code.

Only geometry.
"""

from copy import deepcopy
from robot_controller.constants import TARGET_HYSTERESIS
from robot_controller.constants import *


class Planner:

    def __init__(self):

        self.deadband = PLANNER_DEADBAND
        self.last_requested_target = None
        self.last_goal = None

    def compute_goal(self, current_pose, target_pose):

        goal = deepcopy(target_pose)

        #
# Ignore tiny target changes
#



        p = goal.pose.position

        #
        # Workspace clamp
        #

        p.x = min(max(p.x, MIN_X), MAX_X)
        p.y = min(max(p.y, MIN_Y), MAX_Y)
        p.z = min(max(p.z, MIN_Z), MAX_Z)

        #
        # Deadband
        #

        if self.last_goal is not None:

            dx = p.x - self.last_goal.pose.position.x
            dy = p.y - self.last_goal.pose.position.y
            dz = p.z - self.last_goal.pose.position.z

            dist = (dx * dx + dy * dy + dz * dz) ** 0.5

            if dist < self.deadband:
                return self.last_goal
            
        current = current_pose.pose.position
        target = goal.pose.position

        dx = target.x - current.x
        dy = target.y - current.y
        dz = target.z - current.z

        distance = (dx*dx + dy*dy + dz*dz) ** 0.5

        #
        # Already there?
        #

        if distance < GOAL_REACHED_DISTANCE:
            return current_pose

        #
        # Limit motion per planning cycle
        #

        #
        # Adaptive step size
        #

        step = MAX_STEP_SIZE

        #
        # Slow down as we approach the goal
        #

        if distance < 0.20:
            step = 0.01

        elif distance < 0.40:
            step = 0.02

        elif distance < 0.60:
            step = 0.04

        #
        # Apply step limit
        #

        if distance > step:

            scale = step / distance

            goal.pose.position.x = current.x + dx * scale
            goal.pose.position.y = current.y + dy * scale
            goal.pose.position.z = current.z + dz * scale

        #
        # Remember target and goal
        #

        self.last_requested_target = deepcopy(target_pose)
        self.last_goal = deepcopy(goal)

        return goal
