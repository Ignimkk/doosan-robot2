#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class ColorObjectDetector(Node):
    def __init__(self):
        super().__init__('color_object_detector')

        # 파라미터 파일 로드
        self.calibration_params = self.load_calibration_params(
            "/home/mk/dev_ws/prject/doosan_robotics2/src/doosan-robot2/calibration_params.json"
        )
        self.camera_params = self.load_camera_params(
            "/home/mk/dev_ws/prject/doosan_robotics2/src/doosan-robot2/dsr_sensor/config/MultiMatrix.npz"
        )

        # ROS2 파라미터 초기화
        self.calibration_origin = self.calibration_params["calibration_origin"]
        self.reference_pixel = self.calibration_params["reference_pixel"]
        self.scale = self.calibration_params["scale"]
        self.camera_matrix = self.camera_params["camera_matrix"]
        self.dist_coeffs = self.camera_params["dist_coeffs"]

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Color object detector node initialized.')

    def load_calibration_params(self, filename):
        """JSON 파일에서 캘리브레이션 파라미터 로드"""
        with open(filename, "r") as f:
            params = json.load(f)
        self.get_logger().info(f"Loaded calibration parameters from {filename}.")
        return params

    def load_camera_params(self, filename):
        """NPZ 파일에서 카메라 매트릭스 및 왜곡 계수 로드"""
        data = np.load(filename)
        self.get_logger().info(f"Loaded camera parameters from {filename}.")
        return {
            "camera_matrix": data["camMatrix"],  # 키 이름 수정
            "dist_coeffs": data["distCoef"]      # 키 이름 수정
        }


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 왜곡 보정
        undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

        # 객체 검출
        detected_objects = self.detect_objects(undistorted_image)
        for index, (color_name, center_pixel) in enumerate(detected_objects):
            # 좌표 캘리브레이션
            calibrated_coords = self.calibrate_coordinates(center_pixel)
            self.get_logger().info(f"{color_name} object (Index {index + 1}) detected at calibrated coordinates: {calibrated_coords}")
            
            # 결과 시각화
            cv2.circle(undistorted_image, (int(center_pixel[0]), int(center_pixel[1])), 5, (0, 255, 0), -1)
            cv2.putText(undistorted_image, f"{color_name} ({calibrated_coords[0]:.1f}, {calibrated_coords[1]:.1f}mm)", 
                        (int(center_pixel[0]) + 10, int(center_pixel[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Color Object Detection", undistorted_image)
        cv2.waitKey(1)

    def detect_objects(self, image):
        """색상별 객체 검출"""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_objects = []

        # 노란색 범위
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        detected_objects.extend(self.find_color(hsv_image, yellow_lower, yellow_upper, "Yellow"))

        # 빨간색 범위 (두 범위로 분리됨)
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        detected_objects.extend(self.find_color(hsv_image, red_lower1, red_upper1, "Red"))

        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        detected_objects.extend(self.find_color(hsv_image, red_lower2, red_upper2, "Red"))

        # 파란색 범위
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([140, 255, 255])
        detected_objects.extend(self.find_color(hsv_image, blue_lower, blue_upper, "Blue"))

        return detected_objects

    def find_color(self, hsv_image, lower, upper, color_name):
        """특정 색상 범위 내 객체 탐지"""
        mask = cv2.inRange(hsv_image, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 최소 면적 조건
                x, y, w, h = cv2.boundingRect(contour)
                center_pixel = (x + w / 2, y + h / 2)
                detected_objects.append((color_name, center_pixel))

        return detected_objects

    def calibrate_coordinates(self, object_pixel):
        """객체 중심 픽셀 좌표를 기준 좌표계로 변환"""
        # 증분 계산
        dx_pixel = object_pixel[0] - self.reference_pixel[0]
        dy_pixel = object_pixel[1] - self.reference_pixel[1]

        # 증분 스왑
        dx_swapped, dy_swapped = dy_pixel, dx_pixel

        # 픽셀 증분을 월드 좌표로 변환
        dx_world = dx_swapped * self.scale
        dy_world = dy_swapped * self.scale

        # 캘리브레이션 적용
        calibrated_x = dx_world + self.calibration_origin[0]
        calibrated_y = dy_world + self.calibration_origin[1]

        return (calibrated_x, calibrated_y)

def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
