#!/usr/bin/env python3
"""
RealSense D456 + MediaPipe 3D Pose Publisher
Publishes pose matrix as ROS 2 message
"""

import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np
from mediapipe import solutions
import pyrealsense2 as rs
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class RealsensePoseNode(Node):
    def __init__(self):
        super().__init__('realsense_pose_node')
        
        self.get_logger().info("Initializing RealSense Pose Publisher...")
        
        # =====================================================
        # RealSense Setup
        # =====================================================
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(
            rs.stream.color,
            640,
            480,
            rs.format.bgr8,
            30
        )
        
        config.enable_stream(
            rs.stream.depth,
            640,
            480,
            rs.format.z16,
            30
        )
        
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        depth_profile = profile.get_stream(
            rs.stream.depth
        ).as_video_stream_profile()
        
        self.intrinsics = depth_profile.get_intrinsics()
        
        self.get_logger().info("✓ RealSense initialized")
        
        # =====================================================
        # MediaPipe Setup
        # =====================================================
        self.pose = solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.get_logger().info("✓ MediaPipe initialized")
        
        # =====================================================
        # ROS 2 Publisher
        # =====================================================
        self.pose_publisher = self.create_publisher(
            Float32MultiArray,
            '/realsense/pose_matrix',
            10
        )
        
        self.get_logger().info("✓ Publishing to /realsense/pose_matrix")
        
        # Timer for main loop
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 Hz
        self.frame_count = 0
    
    def get_average_depth(self, depth_frame, px, py, kernel=2):
        """Get averaged depth value around pixel"""
        values = []
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        
        for dx in range(-kernel, kernel + 1):
            for dy in range(-kernel, kernel + 1):
                x = px + dx
                y = py + dy
                
                if x < 0 or x >= width:
                    continue
                if y < 0 or y >= height:
                    continue
                
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    values.append(d)
        
        if len(values) == 0:
            return 0.0
        
        return float(np.mean(values))
    
    def timer_callback(self):
        """Main loop - capture and publish pose"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            
            aligned_frames = self.align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
            
            color_image = np.asanyarray(color_frame.get_data())
            
            rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            results = self.pose.process(rgb)
            
            timestamp = time.time()
            
            # Create 33x6 pose matrix (33 landmarks, 6 values each)
            pose_matrix = np.full((33, 6), np.nan, dtype=np.float32)
            
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                h, w, _ = color_image.shape
                
                for idx, lm in enumerate(landmarks):
                    px = int(lm.x * w)
                    py = int(lm.y * h)
                    
                    if px < 0 or px >= w:
                        continue
                    if py < 0 or py >= h:
                        continue
                    
                    depth = self.get_average_depth(depth_frame, px, py)
                    
                    if depth <= 0:
                        continue
                    
                    point_3d = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics,
                        [px, py],
                        depth
                    )
                    
                    X = point_3d[0]
                    Y = point_3d[1]
                    Z = point_3d[2]
                    
                    visibility = lm.visibility
                    
                    pose_matrix[idx] = [
                        X,
                        Y,
                        Z,
                        visibility,
                        depth,
                        timestamp
                    ]
            
            # Convert to ROS message and publish
            msg = Float32MultiArray()
            msg.data = pose_matrix.flatten().tolist()
            
            # Set layout info
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = "landmarks"
            msg.layout.dim[0].size = 33
            msg.layout.dim[0].stride = 33 * 6
            
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[1].label = "values"
            msg.layout.dim[1].size = 6
            msg.layout.dim[1].stride = 6
            
            self.pose_publisher.publish(msg)
            
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                # Print right wrist info every 30 frames
                right_wrist = pose_matrix[16]
                if not np.isnan(right_wrist[0]):
                    self.get_logger().info(
                        f"Frame {self.frame_count}: RIGHT_WRIST = "
                        f"[X={right_wrist[0]:.3f}, Y={right_wrist[1]:.3f}, "
                        f"Z={right_wrist[2]:.3f}, visibility={right_wrist[3]:.2f}]"
                    )
        
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = RealsensePoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
