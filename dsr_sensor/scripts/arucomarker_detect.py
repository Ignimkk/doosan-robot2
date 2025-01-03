#!/usr/bin/env python3

import os
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Int32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
import tf_transformations
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.marker_size = 20.0  # 마커 크기 (mm 단위)

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

        self.pose_publisher = self.create_publisher(Float32MultiArray, '/aruco_marker_pose', QoSProfile(depth=10))
        self.id_publisher = self.create_publisher(Int32, '/marker/id', QoSProfile(depth=10))
        self.detect_sign_publisher = self.create_publisher(Bool, 'aruco_detect_sign', QoSProfile(depth=10))

        # 이미지 토픽 구독
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        # 마커 상태 관리
        self.marker_detection_count = 0
        self.last_detected_id = None

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
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # 아루코 마커 검출
        marker_corners, marker_IDs, _ = self.detector.detectMarkers(gray_frame)

        if marker_corners and marker_IDs is not None:
            # 현재 감지된 마커 ID와 가장 먼저 감지된 마커 ID 선택
            if self.last_detected_id is None or self.last_detected_id not in marker_IDs:
                self.last_detected_id = marker_IDs[0][0]
                self.marker_detection_count = 0

            marker_index = np.where(marker_IDs == self.last_detected_id)[0][0]
            marker_corner = marker_corners[marker_index]

            # PnP 알고리즘으로 포즈 추정
            ret, rVec, tVec = cv.solvePnP(
                self.get_marker_corners_3d(),
                marker_corner[0].astype(np.float32),
                self.cam_mat,
                self.dist_coef,
            )

            if ret:
                # 동일 마커 검출 횟수 증가
                self.marker_detection_count += 1

                R_marker_camera, _ = cv.Rodrigues(rVec)
                rpy_camera = tf_transformations.euler_from_matrix(R_marker_camera)
                rpy_camera = np.round(np.array(rpy_camera) * (180 / np.pi), 4)

                self.current_xyz_msg = Vector3(
                    x=float(tVec[0][0]),
                    y=float(tVec[1][0]),
                    z=float(tVec[2][0]),
                )
                self.current_rxyz_msg = Vector3(
                    x=rpy_camera[0],
                    y=rpy_camera[1],
                    z=rpy_camera[2],
                )

                self.current_marker_pose = Float32MultiArray()
                self.current_marker_pose.data = [
                    self.current_xyz_msg.x,
                    self.current_xyz_msg.y,
                    self.current_xyz_msg.z,
                    self.current_rxyz_msg.x,
                    self.current_rxyz_msg.y,
                    self.current_rxyz_msg.z,
                ]

                # 30 프레임 이상 동일 마커 감지 시 출력
                if self.marker_detection_count >= 30:
                    self.get_logger().info(
                        f"Marker ID {self.last_detected_id}: Position (x={self.current_xyz_msg.x:.2f}, "
                        f"y={self.current_xyz_msg.y:.2f}, z={self.current_xyz_msg.z:.2f}), "
                        f"Orientation (x={self.current_rxyz_msg.x:.2f}, "
                        f"y={self.current_rxyz_msg.y:.2f}, z={self.current_rxyz_msg.z:.2f})"
                    )
                    self.marker_detection_count = 0
                    self.pose_publisher.publish(self.current_marker_pose)

                    # Publish marker ID
                    marker_id_msg = Int32()
                    marker_id_msg.data = int(self.last_detected_id)
                    self.id_publisher.publish(marker_id_msg)

                    self.detect_sign_publisher.publish(Bool(data=True))

                # 마커 및 좌표 시각화
                cv.aruco.drawDetectedMarkers(frame, [marker_corner])
                cv.drawFrameAxes(
                    frame,
                    self.cam_mat,
                    self.dist_coef,
                    rVec,
                    tVec,
                    50  # 축 길이 (mm)
                )

            # 이미지 표시
            cv.imshow("Aruco Detection", frame)
            cv.waitKey(1)
        else:
            self.last_detected_id = None
            self.marker_detection_count = 0


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
