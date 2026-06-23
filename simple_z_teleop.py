#!/usr/bin/env python3
"""
Simple Z-axis Keyboard Teleoperation for CRX-10iA/L
Tests MoveIt IK with keyboard input (UP/DOWN arrows for Z control)
Runs in RViz simulation mode - NO REAL ROBOT MOTION
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, Constraints
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
import math
import termios
import tty

class SimpleZTeleop(Node):
    def __init__(self):
        super().__init__('simple_z_teleop')
        
        self.get_logger().info("=== Z-Axis Keyboard Teleop (Simulation) ===")
        self.get_logger().info("This will test MoveIt IK without moving the real robot")
        
        # Wait for services
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        
        self.get_logger().info("✓ Connected to MoveIt IK service")
        
        # Parameters
        self.z_min = 0.3
        self.z_max = 2.5
        self.z_step = 0.05
        self.current_z = 1.0  # Start in middle
        
        # Example starting pose (adjust as needed)
        self.start_x = 0.3
        self.start_y = 0.0
        
        self.get_logger().info(f"\nControl Range: Z = {self.z_min}m to {self.z_max}m")
        self.get_logger().info(f"Current Z: {self.current_z:.3f}m\n")
        self.get_logger().info("Press:")
        self.get_logger().info("  UP arrow   -> Increase Z")
        self.get_logger().info("  DOWN arrow -> Decrease Z")
        self.get_logger().info("  Q          -> Quit\n")
    
    def get_key(self):
        """Read single keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def solve_ik(self, x, y, z):
        """Request IK solution from MoveIt"""
        
        # Create target pose with flange DOWN (pitch=90°)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        # Flange DOWN: roll=0, pitch=90°, yaw=0
        q = quaternion_from_euler(0, math.pi/2, 0)
        target_pose.orientation.x = float(q[0])
        target_pose.orientation.y = float(q[1])
        target_pose.orientation.z = float(q[2])
        target_pose.orientation.w = float(q[3])
        
        # Create IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = "manipulator"
        ik_request.pose_stamped.header.frame_id = "base_link"
        ik_request.pose_stamped.pose = target_pose
        ik_request.timeout = rclpy.duration.Duration(seconds=0.2).to_msg()
        
        # Send request
        srv_request = GetPositionIK.Request()
        srv_request.ik_request = ik_request
        
        future = self.ik_client.call_async(srv_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                joint_angles = list(response.solution.joint_state.position)
                return joint_angles
            else:
                self.get_logger().warn(f"IK Failed (error code: {response.error_code.val})")
                return None
        else:
            self.get_logger().warn("IK Service call failed")
            return None
    
    def run(self):
        """Main loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key.lower() == 'q':
                    self.get_logger().info("Quit requested")
                    break
                
                # Handle arrow keys
                if key == '\x1b':  # Escape sequence
                    key2 = self.get_key()
                    if key2 == '[':
                        key3 = self.get_key()
                        
                        if key3 == 'A':  # UP arrow
                            new_z = min(self.z_max, self.current_z + self.z_step)
                            if new_z != self.current_z:
                                self.get_logger().info(f"\n↑ UP: Z {self.current_z:.3f} → {new_z:.3f}m")
                                joints = self.solve_ik(self.start_x, self.start_y, new_z)
                                if joints:
                                    self.current_z = new_z
                                    self.get_logger().info(f"✓ IK Success! Joints: {[f'{j:.2f}°' for j in joints]}")
                                else:
                                    self.get_logger().warn(f"✗ IK Failed for Z={new_z:.3f}m")
                        
                        elif key3 == 'B':  # DOWN arrow
                            new_z = max(self.z_min, self.current_z - self.z_step)
                            if new_z != self.current_z:
                                self.get_logger().info(f"\n↓ DOWN: Z {self.current_z:.3f} → {new_z:.3f}m")
                                joints = self.solve_ik(self.start_x, self.start_y, new_z)
                                if joints:
                                    self.current_z = new_z
                                    self.get_logger().info(f"✓ IK Success! Joints: {[f'{j:.2f}°' for j in joints]}")
                                else:
                                    self.get_logger().warn(f"✗ IK Failed for Z={new_z:.3f}m")
                
                rclpy.spin_once(self, timeout_sec=0.01)
        
        except KeyboardInterrupt:
            self.get_logger().info("\nKeyboardInterrupt")
        finally:
            self.get_logger().info("Teleop node finished")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleZTeleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
