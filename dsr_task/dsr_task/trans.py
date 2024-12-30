#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np
import tf_transformations

class TFTransformNode(Node):
    def __init__(self):
        super().__init__('tf_transform_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose',
            self.pose_callback,
            QoSProfile(depth=10)
        )

        self.transformed_pose_publisher = self.create_publisher(
            PoseStamped,
            '/aruco_marker_pose_base_link',
            QoSProfile(depth=10)
        )

    def pose_callback(self, msg):
        try:
            source_frame = 'link_6'
            target_frame = 'base_link'

            self.get_logger().info(f"Attempting transform from {source_frame} to {target_frame}")

            # Transform from link_6 to base_link
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # Use the latest available transform
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Extract translation and rotation from msg
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

            # Create rotation matrix for -90 degrees around z-axis
            rotation_z_neg_90 = tf_transformations.quaternion_matrix(
                tf_transformations.quaternion_from_euler(0, 0, -1.5708)
            )[:3, :3]

            # Rotate position by -90 degrees around z-axis
            rotated_position = rotation_z_neg_90.dot(position)

            self.get_logger().info(f"Rotated Position (after -90 deg): {rotated_position}")

            # Apply translation (link_6 to base_link)
            final_position = rotated_position + np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])

            self.get_logger().info(f"Final Position in base_link: {final_position}")

            # Combine rotations
            additional_rotation = tf_transformations.quaternion_from_euler(0, 0, 1.5708)
            original_rotation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            corrected_rotation = tf_transformations.quaternion_multiply(original_rotation, additional_rotation)

            # Publish transformed pose
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            transformed_pose.pose.position.x = final_position[0]
            transformed_pose.pose.position.y = final_position[1]
            transformed_pose.pose.position.z = final_position[2]
            transformed_pose.pose.orientation.x = corrected_rotation[0]
            transformed_pose.pose.orientation.y = corrected_rotation[1]
            transformed_pose.pose.orientation.z = corrected_rotation[2]
            transformed_pose.pose.orientation.w = corrected_rotation[3]

            self.transformed_pose_publisher.publish(transformed_pose)
            self.get_logger().info(f"Transformed pose published: {transformed_pose}")

        except TransformException as ex:
            self.get_logger().error(f"TransformException: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = TFTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
