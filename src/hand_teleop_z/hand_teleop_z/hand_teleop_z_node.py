#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveGroupInterface
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
import math
import sys
import termios
import tty

class HandTeleopZNode(Node):
    """Simple Z-axis keyboard teleoperation for CRX-10iA/L"""
    
    def __init__(self):
        super().__init__('hand_teleop_z_node')
        
        self.get_logger().info("Initializing Hand Teleop Z Node...")
        
        # Initialize MoveIt
        self.move_group = MoveGroupInterface(
            name="manipulator",
            robot_description="/robot_description",
            ns=self.get_namespace()
        )
        
        # Get current pose (this will be our starting point)
        self.current_pose = self.move_group.get_current_pose()
        self.get_logger().info(f"Current pose: {self.current_pose}")
        
        # Z-axis bounds (in meters)
        self.z_min = 0.3
        self.z_max = 2.5
        self.z_step = 0.05  # 5cm per key press
        
        # Current Z position
        self.current_z = self.current_pose.position.z
        
        self.get_logger().info(f"Starting Z position: {self.current_z:.3f}m")
        self.get_logger().info("Keyboard Controls:")
        self.get_logger().info("  UP arrow / W    -> Move gripper UP (increase Z)")
        self.get_logger().info("  DOWN arrow / S  -> Move gripper DOWN (decrease Z)")
        self.get_logger().info("  Q               -> Quit")
        
    def get_key(self):
        """Read keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def move_to_z(self, z_target):
        """Move end-effector to target Z while keeping flange DOWN"""
        
        # Clamp Z to valid range
        z_target = max(self.z_min, min(self.z_max, z_target))
        
        if abs(z_target - self.current_z) < 0.001:
            self.get_logger().info("Z position unchanged, skipping move")
            return False
        
        # Create target pose: same X,Y as current, new Z
        target_pose = Pose()
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y
        target_pose.position.z = z_target
        
        # Set orientation: flange DOWN (roll=0, pitch=pi/2, yaw=0)
        # This means end-effector normal points downward
        q = quaternion_from_euler(0, math.pi/2, 0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        
        self.get_logger().info(f"Moving to Z = {z_target:.3f}m")
        
        # Try to move to target pose
        success = self.move_group.go_to_pose_goal(target_pose, wait=True)
        
        if success:
            self.current_z = z_target
            self.current_pose = self.move_group.get_current_pose()
            self.get_logger().info(f"✓ Reached Z = {self.current_z:.3f}m")
            return True
        else:
            self.get_logger().warn(f"✗ Failed to reach Z = {z_target:.3f}m")
            return False
    
    def run(self):
        """Main keyboard loop"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Ready for keyboard input!")
        self.get_logger().info("="*50 + "\n")
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'q' or key == 'Q':
                    self.get_logger().info("Quit requested")
                    break
                
                elif key == '\x1b':  # Escape sequence for arrow keys
                    key2 = self.get_key()
                    if key2 == '[':
                        key3 = self.get_key()
                        
                        if key3 == 'A':  # UP arrow
                            self.get_logger().info("UP arrow pressed")
                            self.move_to_z(self.current_z + self.z_step)
                        
                        elif key3 == 'B':  # DOWN arrow
                            self.get_logger().info("DOWN arrow pressed")
                            self.move_to_z(self.current_z - self.z_step)
                
                elif key == 'w' or key == 'W':  # UP
                    self.get_logger().info("W pressed (UP)")
                    self.move_to_z(self.current_z + self.z_step)
                
                elif key == 's' or key == 'S':  # DOWN
                    self.get_logger().info("S pressed (DOWN)")
                    self.move_to_z(self.current_z - self.z_step)
                
                rclpy.spin_once(self, timeout_sec=0.1)
        
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt - Shutting down")
        finally:
            self.get_logger().info("Hand Teleop Z Node stopped")

def main(args=None):
    rclpy.init(args=args)
    
    node = HandTeleopZNode()
    node.run()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
