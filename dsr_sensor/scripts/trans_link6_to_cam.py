#!/usr/bin/env python3

from dsr_msgs2.srv import GetCurrentPosx, Trans
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# for single robot
ROBOT_ID = ""
ROBOT_MODEL = "a0509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class CameraOffsetNode(Node):
    def __init__(self):
        super().__init__('camera_offset_node')

        # 카메라 위치 퍼블리셔 생성
        self.camera_pose_pub = self.create_publisher(PoseStamped, 'camera_pose', 10)

        # Aruco 마커 감지 토픽 구독
        self.aruco_detect_sub = self.create_subscription(
            Bool, 
            'aruco_detect_sign', 
            self.aruco_detect_callback, 
            QoSProfile(depth=10)
        )
        self.aruco_detected = False

        # GetCurrentPosx 및 Trans 서비스 클라이언트 생성
        self.get_pos_client = self.create_client(GetCurrentPosx, 'aux_control/get_current_posx')
        self.trans_client = self.create_client(Trans, '/motion/trans')

        while not self.get_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetCurrentPosx service...')

        while not self.trans_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Trans service...')

    def aruco_detect_callback(self, msg):
        self.aruco_detected = msg.data

    def timer_callback(self):
        # GetCurrentPosx 요청 메시지 생성
        get_pos_request = GetCurrentPosx.Request()
        get_pos_request.ref = 0  # DR_BASE 기준으로 요청

        # GetCurrentPosx 서비스 호출
        # self.get_logger().info("Sending GetCurrentPosx service request...")
        get_pos_future = self.get_pos_client.call_async(get_pos_request)
        rclpy.spin_until_future_complete(self, get_pos_future)

        if get_pos_future.result() is not None:
            get_pos_response = get_pos_future.result()
            if get_pos_response.success:
                # 엔드이펙터의 현재 위치
                end_effector_pos = get_pos_response.task_pos_info[0].data[:6]
                self.get_logger().info(f"End Effector Position: {end_effector_pos}")

                # Trans 요청 메시지 생성
                trans_request = Trans.Request()
                trans_request.pos = end_effector_pos
                trans_request.delta = [-80.0, 30.0, 25.0, 0.0, 0.0, -1.57]  # 카메라 옵셋
                trans_request.ref = 1  # DR_TOOL
                trans_request.ref_out = 0  # DR_BASE

                # Trans 서비스 호출
                # self.get_logger().info("Sending Trans service request...")
                trans_future = self.trans_client.call_async(trans_request)
                rclpy.spin_until_future_complete(self, trans_future)

                if trans_future.result() is not None:
                    trans_response = trans_future.result()
                    if trans_response.success:
                        # 카메라의 위치 변환 및 퍼블리싱
                        camera_pos_mm = [round(x, 5) for x in trans_response.trans_pos]
                        self.get_logger().info(f"Camera Position: {camera_pos_mm}")

                        # PoseStamped 메시지 생성 및 퍼블리싱
                        camera_pose_msg = PoseStamped()
                        camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        camera_pose_msg.header.frame_id = "base_link"
                        camera_pose_msg.pose.position = Point(x=camera_pos_mm[0] / 1000.0, 
                                                              y=camera_pos_mm[1] / 1000.0, 
                                                              z=camera_pos_mm[2] / 1000.0)
                        camera_pose_msg.pose.orientation = Quaternion(x=end_effector_pos[3], y=end_effector_pos[4], z=end_effector_pos[5], w=1.0)  # Orientation 기본값
                        self.camera_pose_pub.publish(camera_pose_msg)

                    else:
                        self.get_logger().error("Trans service call failed: success=False")
                else:
                    self.get_logger().error("Trans service call failed: no result")
            else:
                self.get_logger().error("GetCurrentPosx service call failed: success=False")
        else:
            self.get_logger().error("GetCurrentPosx service call failed: no result")

def main(args=None):
    rclpy.init(args=args)
    node = CameraOffsetNode()
    DR_init.__dsr__node = node
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.aruco_detected:
                node.timer_callback()
                node.aruco_detected = False
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user (KeyboardInterrupt). Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
