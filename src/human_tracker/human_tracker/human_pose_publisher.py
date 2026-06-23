#!/usr/bin/env python3

import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import rclpy

try:
    from human_tracker.rviz_skeleton import SkeletonPublisher
except ModuleNotFoundError:
    from rviz_skeleton import SkeletonPublisher


def get_average_depth(depth_frame, px, py, kernel=2):
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

            depth = depth_frame.get_distance(x, y)

            if depth > 0:
                values.append(depth)

    if not values:
        return 0.0

    return float(np.mean(values))


def build_pose_3d(results, color_image, depth_frame, intrinsics):
    pose_3d = np.full((33, 3), np.nan, dtype=np.float32)

    if not results.pose_landmarks:
        return pose_3d, 0

    height, width, _ = color_image.shape
    landmarks = results.pose_landmarks.landmark
    valid_count = 0

    for idx, landmark in enumerate(landmarks):
        px = int(landmark.x * width)
        py = int(landmark.y * height)

        if px < 0 or px >= width:
            continue

        if py < 0 or py >= height:
            continue

        depth = get_average_depth(depth_frame, px, py)

        if depth <= 0:
            continue

        point = rs.rs2_deproject_pixel_to_point(
            intrinsics,
            [px, py],
            depth,
        )

        pose_3d[idx] = [
            point[0],
            point[1],
            point[2],
        ]
        valid_count += 1

    return pose_3d, valid_count


def main():
    if not hasattr(mp, "solutions"):
        raise RuntimeError(
            "This script needs the MediaPipe Solutions API. "
            "Install a compatible version, for example: "
            "python -m pip install 'mediapipe==0.10.21'"
        )

    rclpy.init()
    skeleton_pub = SkeletonPublisher()

    print("ROS node created")
    print(f"node name: {skeleton_pub.get_name()}")
    print("publishing MarkerArray on: /human_markers")

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        640,
        480,
        rs.format.bgr8,
        30,
    )

    config.enable_stream(
        rs.stream.depth,
        640,
        480,
        rs.format.z16,
        30,
    )

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    color_profile = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()
    intrinsics = color_profile.get_intrinsics()

    mp_pose = mp.solutions.pose
    mp_draw = mp.solutions.drawing_utils

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    frame_count = 0
    publish_count = 0
    last_debug_time = time.time()

    print("\nRunning...")
    print("ESC = Quit\n")

    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb_image)

            pose_3d, valid_count = build_pose_3d(
                results,
                color_image,
                depth_frame,
                intrinsics,
            )

            markers_valid, skeleton_lines = skeleton_pub.publish_points(
                pose_3d,
                mp_pose.POSE_CONNECTIONS,
            )
            publish_count += 1
            rclpy.spin_once(skeleton_pub, timeout_sec=0.0)

            if results.pose_landmarks:
                mp_draw.draw_landmarks(
                    color_image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                )

            frame_count += 1
            now = time.time()
            elapsed = now - last_debug_time

            if elapsed >= 1.0:
                print(
                    "pose loop: "
                    f"frames={frame_count} "
                    f"published={publish_count} "
                    f"pose_detected={results.pose_landmarks is not None} "
                    f"valid_3d_points={valid_count} "
                    f"markers_valid={markers_valid} "
                    f"skeleton_lines={skeleton_lines} "
                    f"rate={publish_count / elapsed:.1f} Hz"
                )
                frame_count = 0
                publish_count = 0
                last_debug_time = now

            cv2.imshow("RealSense MediaPipe Pose", color_image)

            if cv2.waitKey(1) == 27:
                break

    finally:
        pose.close()
        pipeline.stop()
        cv2.destroyAllWindows()
        skeleton_pub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
