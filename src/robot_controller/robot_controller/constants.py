"""
Project-wide constants.

Keeping these in one place makes tuning the controller much easier.
"""

#
# ROS Topics
#

AVOIDANCE_TOPIC = "/avoidance_target"

#
# MoveIt
#

MOVE_ACTION = "/move_action"

PLANNING_GROUP = "manipulator"

END_EFFECTOR_LINK = "flange"

REFERENCE_FRAME = "world"

#
# Planner
#

PLANNING_RATE = 5.0        # Hz

GOAL_TOLERANCE = 0.01      # metres

PLANNER_DEADBAND = 0.02
#
# Workspace limits
#

MIN_X = 0.30
MAX_X = 1.10

MIN_Y = -0.80
MAX_Y = 0.80

MIN_Z = 0.10
MAX_Z = 1.20

# -----------------------------
# Motion
# -----------------------------

MAX_STEP_SIZE = 0.05      # 5 cm per planning cycle

GOAL_REACHED_DISTANCE = 0.01

# -----------------------------
# Planner
# -----------------------------

TARGET_HYSTERESIS = 0.03      # meters (2 cm)

# -----------------------------
# Timing
# -----------------------------

MAX_PLANNING_RATE = 5.0      # Hz