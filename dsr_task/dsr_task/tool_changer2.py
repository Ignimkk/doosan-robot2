import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
import sys
import time

# for single robot
ROBOT_ID = ""
ROBOT_MODEL = "a0509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class ToolChangerIn(Node):
    def __init__(self):
        super().__init__('tool_changer_in', namespace=ROBOT_ID)
        self.subscriber = self.create_subscription(Bool, 'gripper_take_on', self.gripper_callback, 10)
        self.gripper_activated = False

    def gripper_callback(self, msg):
        """Callback to handle gripper_take_on topic."""
        if msg.data:
            self.gripper_activated = True
            self.get_logger().info("Gripper Take On signal received.")

    def execute_motions(self):
        try:
            from DSR_ROBOT2 import movej, movel, set_velx, set_accx, DR_BASE, DR_TOOL, DR_MV_MOD_ABS, DR_MV_MOD_REL
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        # Set values
        set_velx(100, 20)
        set_accx(1000, 40)

        IN_J01 = [103.83, -0.26, 100.17, 0.0, 80.09, 103.23]
        IN_L02 = [-85.990, 349.210, 336.120, 104.84, 180.0, 104.23]
        IN_L03 = [0.0, -150.0, 0.0, 0.0, 0.0, 0.0]
        IN_J04 = [180.0, 0.0, 90.0, 0.0, 90.0, 90.0]

        # Execute motions
        self.get_logger().info("Executing motions...")
        movej(IN_J01, vel=60, acc=30, time=0, mod=DR_MV_MOD_ABS)
        time.sleep(1)
        movel(IN_L02, vel=100, acc=1000, time=0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        time.sleep(1)
        movel(IN_L03, vel=60, acc=30, time=0, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        time.sleep(1)
        movej(IN_J04, vel=60, acc=30, time=0)
        self.get_logger().info("Motion execution complete. Shutting down node.")

def main(args=None):
    rclpy.init(args=args)
    node = ToolChangerIn()

    DR_init.__dsr__node = node

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.gripper_activated:
                node.execute_motions()
                break
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
