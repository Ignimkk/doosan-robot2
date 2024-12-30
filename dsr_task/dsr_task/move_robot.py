import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from dsr_msgs2.msg import ServolStream  

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(ServolStream, '/servol_stream', 10)  
        self.marker_id_sub = self.create_subscription(Bool, '/test_servol', self.listener_callback, 10)
        # self.timer = self.create_timer(1.0, self.publish_command)  # 1초마다 발행

    def listener_callback(self, msg):
        if msg.data:  
            self.publish_command()
            
    
    def publish_command(self):
        msg = ServolStream()
        msg.pos = [-85.990, 349.210, 450.120, 104.84, 180.0, 104.23]  # 목표 위치 (예제 값)
        msg.vel = [300.0, 100.0]  
        msg.acc = [505.0, 100.0]  
        msg.time = 0.0 

        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped manually.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
