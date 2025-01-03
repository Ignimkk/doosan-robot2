import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
import time

# for single robot
ROBOT_ID = ""
ROBOT_MODEL = "a0509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class Homming(Node):
    def __init__(self):
        super().__init__('homming', namespace=ROBOT_ID)
        self.subscriber = self.create_subscription(String, 'homming', self.homming_callback, 10)
        self.get_logger().info("homming node initialized.")
        self.msg_data = "None"

        
    def homming_excute(self):
        try:
            from DSR_ROBOT2 import movej, DR_MV_MOD_ABS
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return
        
        HOME_J01 = [180.0, 0.0, 90.0, 0.0, 90.0, 90.0]
        movej(HOME_J01, vel=60, acc=30, time=0, mod=DR_MV_MOD_ABS)

        self.get_logger().info("homming motions complete.")
        
    def homming_callback(self, msg):
        """Callback to handle gripper_take_on topic."""
        if msg.data == "homming":
            self.get_logger().info("Received 'homming' command. Activating homming.")
            self.msg_data = 'HOME'
        else:
            self.get_logger().warning(f"Invalid command received: {msg.data}")

def main(args=None):
    import rclpy
    rclpy.init(args=args)  # ROS2 초기화
    node = Homming()  
    DR_init.__dsr__node = node

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.msg_data == 'HOME':
                node.homming_excute()
                node.msg_data = 'None'  
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user (KeyboardInterrupt). Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
