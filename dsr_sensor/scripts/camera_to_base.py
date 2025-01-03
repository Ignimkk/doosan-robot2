#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, rotation_matrix

class MarkerPoseTransformer(Node):
    def __init__(self):
        super().__init__('marker_pose_transformer')

        # Publisher for transformed marker pose
        self.marker_pose_pub = self.create_publisher(Float32MultiArray, '/marker/pose', 10)

        self.marker_pose_sub = self.create_subscription(
            Float32MultiArray,
            'aruco_marker_pose',
            self.marker_pose_callback,
            10
        )

        # TF2 listener for base_link to link_6 transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform_matrix(self, source_frame, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

            rotation_matrix_tf = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

            transform_matrix = np.dot(translation_matrix, rotation_matrix_tf)

            # Apply additional Z-axis rotation (90 degrees)
            z_rotation = rotation_matrix(np.radians(-90), [0, 0, 1])  # Z축 기준 90도 회전
            transform_matrix = np.dot(transform_matrix, z_rotation)

            return transform_matrix
        except Exception as e:
            self.get_logger().error(f"Error retrieving transform: {e}")
            return None

    def marker_pose_callback(self, msg):
        T_base_to_link6 = self.get_transform_matrix('link_6', 'base_link')
        if T_base_to_link6 is None:
            self.get_logger().warning("Transformation from base_link to link_6 not available. Skipping marker pose transformation.")
            return

        try:
            # Marker pose in camera frame (aruco_marker_pose provides x, y, z in mm)
            marker_pose_in_camera = np.array([msg.data[0] / 1000, msg.data[1] / 1000, msg.data[2] / 1000, 1.0])

            # Transform to base_link frame
            marker_pose_in_base = np.dot(T_base_to_link6, marker_pose_in_camera)

            # Apply an offset of 8cm along the x-axis of link_6
            offset_matrix = np.eye(4)
            offset_matrix[0, 3] = -0.08  # 8cm offset along x-axis
            offset_matrix[1, 3] = -0.03  # -3cm offset along y-axis
            offset_matrix[2, 3] = 0.024  # 2.4cm offset along z-axis

            marker_pose_with_offset = np.dot(offset_matrix, marker_pose_in_base)

            # Extract transformed x, y, z with offset and convert to mm
            x_base, y_base, z_base = marker_pose_with_offset[:3] * 1000  # Convert to mm

            # Create Float32MultiArray message
            marker_pose_msg = Float32MultiArray()
            marker_pose_msg.data = [x_base, y_base, z_base]

            # Publish transformed pose
            self.marker_pose_pub.publish(marker_pose_msg)
            self.get_logger().info(f"Published marker pose in base_link with offset: x={x_base:.3f}, y={y_base:.3f}, z={z_base:.3f}")

        except Exception as e:
            self.get_logger().error(f"Error transforming marker pose: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
