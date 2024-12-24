import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
import sys
import time

# for single robot
ROBOT_ID = ""
ROBOT_MODEL = "a0509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control', namespace=ROBOT_ID)
        self.subscription = self.create_subscription(String, 'gripper_command', self.gripper_callback, 10)
        self.gripper_change_publisher = self.create_publisher(String, '/gripper_change', 10)
        self.get_logger().info("Gripper control node initialized.")
        self.msg_data = "None"

    def gripper_callback(self, msg):
        if msg.data == 'close':  # True
            self.get_logger().info("Received True command. Closing gripper.")
            self.msg_data = 'CLOSE'
        else:  # False
            self.get_logger().info("Received False command. Opening gripper.")
            self.msg_data = 'OPEN'

    def check_tool_weight(self):
        try:
            from DSR_ROBOT2 import get_workpiece_weight
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return -1

        weight = get_workpiece_weight()
        if weight == -1:
            self.get_logger().error("Failed to retrieve tool weight.")
        else:
            self.get_logger().info(f"Current tool weight: {weight}kg")
        return weight

    def gripper_close(self):
        try:
            from DSR_ROBOT2 import set_tool_digital_output
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        weight = self.check_tool_weight()
        if weight < 0.6:
            self.get_logger().warning("Tool weight is below 0.6kg. Requesting gripper change.")
            if not self.publish_gripper_change():
                self.get_logger().error("Gripper change request failed. Aborting gripper close.")
                return
            
        self.get_logger().info("Executing gripper_close digital output commands...")
        set_tool_digital_output(1, 0)  # Set port 1 to 0
        time.sleep(0.1)
        set_tool_digital_output(2, 0)  # Set port 2 to 0
        time.sleep(0.1)
        set_tool_digital_output(1, 1)  # Set port 1 to 1
        self.get_logger().info("gripper_close commands complete.")

    def gripper_open(self):
        try:
            from DSR_ROBOT2 import set_tool_digital_output
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        # Execute digital output commands for opening gripper
        self.get_logger().info("Executing gripper_open digital output commands...")
        set_tool_digital_output(1, 0)  # Set port 1 to 0
        time.sleep(0.1)
        set_tool_digital_output(2, 0)  # Set port 2 to 0
        time.sleep(0.1)
        set_tool_digital_output(2, 1)  # Set port 2 to 1
        self.get_logger().info("gripper_open commands complete.")

    def publish_gripper_change(self):
        """Publish a request to the /gripper_change topic."""
        msg2 = String()
        msg2.data = 'on'
        self.gripper_change_publisher.publish(msg2)
        self.get_logger().info("Published gripper change request to /gripper_change topic.")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()

    DR_init.__dsr__node = node

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.msg_data == 'CLOSE':
                node.gripper_close()
                node.msg_data = 'None'  
            elif node.msg_data == 'OPEN':
                node.gripper_open()
                node.msg_data = 'None'  
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
