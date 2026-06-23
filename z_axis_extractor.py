#!/usr/bin/env python3
"""
Simple Z-Axis Extractor from RealSense Pose Matrix
Extracts right wrist (landmark 16) Z position
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np

class ZAxisExtractor(Node):
    def __init__(self):
        super().__init__('z_axis_extractor')
        
        # Subscribe to pose matrix
        self.pose_sub = self.create_subscription(
            Float32MultiArray,
            '/realsense/pose_matrix',
            self.pose_callback,
            10
        )
        
        # Publish Z position
        self.z_pub = self.create_publisher(
            Float32,
            '/realsense/right_wrist_z',
            10
        )
        
        self.get_logger().info("Z-Axis Extractor ready")
        self.get_logger().info("  Input: /realsense/pose_matrix")
        self.get_logger().info("  Output: /realsense/right_wrist_z")
    
    def pose_callback(self, msg):
        """Extract right wrist Z position"""
        # Reshape data back to (33, 6)
        data = np.array(msg.data, dtype=np.float32).reshape(33, 6)
        
        # Get right wrist (landmark 16)
        right_wrist = data[16]
        
        # Check if valid (not NaN)
        if not np.isnan(right_wrist[0]):
            z_position = float(right_wrist[2])  # Z is index 2
            visibility = float(right_wrist[3])  # Visibility is index 3
            
            # Only publish if visible
            if visibility > 0.5:
                msg_z = Float32()
                msg_z.data = z_position
                self.z_pub.publish(msg_z)
                
                self.get_logger().debug(
                    f"Right Wrist Z: {z_position:.3f}m (visibility: {visibility:.2f})"
                )

def main(args=None):
    rclpy.init(args=args)
    node = ZAxisExtractor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
