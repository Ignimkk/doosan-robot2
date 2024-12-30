

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import Trans
from geometry_msgs.msg import PoseStamped

class TransformClient(Node):
    def __init__(self):
        super().__init__('transform_client')

        self.client = self.create_client(Trans, '/trans')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the trans service...')
        self.get_logger().info('Trans service is available.')

        # ArUco 마커 포즈 구독
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.get_logger().info(f"Received marker pose: {msg}")
        
        # 메시지의 좌표로 좌표 변환 요청
        pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ]
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ref = 1  # DR_TOOL (Camera 좌표계)
        ref_out = 0  # DR_BASE (Base 좌표계)

        self.send_request(pos, delta, ref, ref_out)

    def send_request(self, pos, delta, ref, ref_out):
        request = Trans.Request()
        request.pos = pos
        request.delta = delta
        request.ref = ref
        request.ref_out = ref_out

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Transformed Position: {response.trans_pos}, Success: {response.success}")
        else:
            self.get_logger().error('Service call failed!')

def main(args=None):
    rclpy.init(args=args)
    node = TransformClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
