#!/usr/bin/env python3

import os
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.marker_size = 50.0  # 마커 크기 (mm 단위)

        try:
            # 패키지에서 캘리브레이션 데이터 로드
            package_share_directory = get_package_share_directory('dsr_sensor')
            calib_data_path = os.path.join(package_share_directory, 'config', 'MultiMatrix.npz')
            
            calib_data = np.load(calib_data_path)
            self.cam_mat = calib_data["camMatrix"]
            self.dist_coef = calib_data["distCoef"]
            
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
            self.parameters = cv.aruco.DetectorParameters()
            self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
            
            self.get_logger().info('Aruco detector initialized successfully.')
        
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Aruco detector: {e}")
            raise
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_marker_pose', QoSProfile(depth=10))

        # 이미지 토픽 구독
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            qos_profile
        )

    def get_marker_corners_3d(self):
        half_size = self.marker_size / 2.0
        return np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
    
    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # 아루코 마커 검출
            marker_corners, marker_IDs, _ = self.detector.detectMarkers(gray_frame)

            if marker_corners and marker_IDs is not None:
                for marker_corner, marker_id in zip(marker_corners, marker_IDs.flatten()):
                    # 2D 코너 좌표를 1D 배열로 변환
                    marker_corner_2d = marker_corner.reshape(-1, 2).astype(np.float32)

                    # PnP 알고리즘을 사용하여 포즈 추정
                    ret, rVec, tVec = cv.solvePnP(
                        self.get_marker_corners_3d(), 
                        marker_corner_2d, 
                        self.cam_mat, 
                        self.dist_coef
                    )

                    if ret:
                        # PoseStamped 메시지 생성
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = "camera_frame"
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.pose.position.x = tVec[0][0] / 1000.0  # mm -> m
                        pose_msg.pose.position.y = tVec[1][0] / 1000.0
                        pose_msg.pose.position.z = tVec[2][0] / 1000.0
                        pose_msg.pose.orientation.x = rVec[0][0]
                        pose_msg.pose.orientation.y = rVec[1][0]
                        pose_msg.pose.orientation.z = rVec[2][0]
                        pose_msg.pose.orientation.w = 1.0  # Rotation as needed

                        # 퍼블리시
                        self.pose_publisher.publish(pose_msg)
                        self.get_logger().info(f"Published marker pose: {pose_msg}")
            # 결과 이미지 시각화
            cv.imshow("Aruco Detection", frame)
            cv.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
