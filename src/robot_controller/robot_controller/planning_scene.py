#!/usr/bin/env python3
import time
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import CollisionObject

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from std_msgs.msg import Header


class PlanningSceneManager:

    def __init__(self, node: Node):

        self.node = node

        #
        # Planning Scene Publisher
        #
        self.scene_pub = node.create_publisher(
            PlanningScene,
            "/planning_scene",
            10,
        )

        self.node.get_logger().info(
            "Planning Scene Manager Ready."
        )

    ########################################################

    def publish_scene(self, collision_object):

        scene = PlanningScene()

        #
        # Update existing planning scene
        #
        scene.is_diff = True

        scene.world.collision_objects.append(
            collision_object
        )

        self.scene_pub.publish(scene)

    ########################################################

    def add_sphere(
        self,
        object_id,
        x,
        y,
        z,
        radius,
        frame="world",
    ):

        collision = CollisionObject()

        collision.id = object_id

        collision.header = Header()

        collision.header.frame_id = frame

        collision.operation = CollisionObject.ADD

        #
        # Sphere geometry
        #
        sphere = SolidPrimitive()

        sphere.type = SolidPrimitive.SPHERE

        sphere.dimensions = [radius]

        collision.primitives.append(
            sphere
        )

        #
        # Sphere pose
        #
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.w = 1.0

        collision.primitive_poses.append(
            pose
        )

        self.node.get_logger().info(
            f"Added sphere '{object_id}' "
            f"at ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        self.publish_scene(collision)

    ########################################################
    ########################################################

    def update_sphere(
        self,
        object_id,
        x,
        y,
        z,
        radius,
        frame="world",
    ):

        #
        # Publishing an ADD operation with the
        # same object ID updates the object.
        #

        self.add_sphere(
            object_id=object_id,
            x=x,
            y=y,
            z=z,
            radius=radius,
            frame=frame,
        )


    def remove_object(self, object_id):

        collision = CollisionObject()

        collision.id = object_id

        collision.header.frame_id = "world"

        collision.operation = CollisionObject.REMOVE

        self.publish_scene(collision)

    ########################################################

    def clear_scene(self):

        collision = CollisionObject()

        collision.operation = CollisionObject.REMOVE

        collision.id = ""

        self.publish_scene(collision)

    def wait_for_subscribers(self, timeout=10.0):

        start = time.time()

        while self.scene_pub.get_subscription_count() == 0:

            if time.time() - start > timeout:

                self.node.get_logger().warn(
                    "Timed out waiting for Planning Scene subscriber."
                )

                return False

            self.node.get_logger().info(
                "Waiting for Planning Scene subscriber..."
            )

            time.sleep(0.5)

        return True