#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import rclpy
from rclpy.node import Node

class CameraToBaseTransformer(Node):
    def __init__(self):
        super().__init__('camera_to_base_transformer')

        # 카메라 위치 및 자세를 구독
        self.camera_pose_sub = self.create_subscription(
            PoseStamped,
            'camera_pose',
            self.camera_pose_callback,
            10
        )

        # 아루코 마커 위치를 구독
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            'aruco_marker_pose',
            self.aruco_pose_callback,
            10
        )

        self.camera_position = None  # 카메라 위치
        self.camera_orientation = None  # 카메라 자세

    def camera_pose_callback(self, msg):
        self.get_logger().info("Received camera pose")
        self.camera_position = [
            msg.pose.position.x * 1000,  # Convert to mm
            msg.pose.position.y * 1000,
            msg.pose.position.z * 1000
        ]
        self.camera_orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

    def aruco_pose_callback(self, msg):
        self.get_logger().info("Received Aruco marker pose")
        if self.camera_position is None or self.camera_orientation is None:
            self.get_logger().warning("Camera pose is not yet available.")
            return

        aruco_in_base = self.convert_to_base_frame(self.camera_position, self.camera_orientation, msg)
        self.get_logger().info(f"Aruco Marker Position in Base Frame (mm): {aruco_in_base}")
        self.camera_position = None
        self.camera_orientation = None

    @staticmethod
    def convert_to_base_frame(camera_position, camera_orientation, aruco_pose):
        tx, ty, tz = np.array(camera_position) / 1000.0  # mm -> m
        roll, pitch, yaw = np.radians(camera_orientation[:3])  # degrees -> radians

        # 회전 행렬 (Roll, Pitch, Yaw 순서)
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        R_camera = R_z @ R_y @ R_x

        # 변환 행렬 (4x4)
        T_camera_to_base = np.eye(4)
        T_camera_to_base[:3, :3] = R_camera
        T_camera_to_base[:3, 3] = [tx, ty, tz]

        # 2. 아루코 마커 좌표를 동차 좌표계로 변환
        aruco_position = np.array([aruco_pose.pose.position.x / 1000.0,
                                    aruco_pose.pose.position.y / 1000.0,
                                    aruco_pose.pose.position.z / 1000.0,
                                    1.0])

        # 3. 변환 적용
        aruco_position_in_base = T_camera_to_base @ aruco_position

        # 4. 결과 반환
        return aruco_position_in_base[:3] * 1000  # m -> mm

def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
