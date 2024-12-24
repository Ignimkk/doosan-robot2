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

class ToolChanger(Node):
    def __init__(self):
        super().__init__('tool_changer', namespace=ROBOT_ID)
        self.subscriber = self.create_subscription(String, 'gripper_change', self.gripper_callback, 10)
        self.gripper_command_publisher = self.create_publisher(String, 'gripper_command', 10)
        self.get_logger().info("ToolChanger node initialized.")
        self.msg_data = "None"
    def gripper_callback(self, msg):
        """Callback to handle gripper_take_on topic."""
        if msg.data == "on":
            self.get_logger().info("Received 'on' command. Activating gripper_put_on.")
            self.msg_data = 'ON'
            # self.gripper_put_on()
        elif msg.data == "off":
            self.get_logger().info("Received 'off' command. Activating gripper_put_off.")
            self.msg_data = 'OFF'
            # self.gripper_put_off()
        elif msg.data == "oon":
            self.get_logger().info("Received 'oon' command. Activating gripper_put_on_execute.")
            self.msg_data = 'OON'
        else:
            self.get_logger().warning(f"Invalid command received: {msg.data}")

    def gripper_put_on(self):
        try:
            from DSR_ROBOT2 import movej, movel, set_velx, set_accx, DR_BASE, DR_TOOL, DR_MV_MOD_ABS, DR_MV_MOD_REL
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        # Set values
        set_velx(100, 20)
        set_accx(1000, 40)

        ON_J01 = [103.83, -0.26, 100.17, 0.0, 80.09, 103.23]
        ON_L02 = [-85.990, 349.210, 336.120, 104.84, 180.0, 104.23]
        ON_L03 = [0.0, -150.0, 0.0, 0.0, 0.0, 0.0]
        ON_J04 = [180.0, 0.0, 90.0, 0.0, 90.0, 90.0]

        # Execute motions
        self.get_logger().info("Executing gripper_put_on motions...")
        movej(ON_J01, vel=60, acc=30, time=0, mod=DR_MV_MOD_ABS)
        time.sleep(1)
        movel(ON_L02, vel=100, acc=1000, time=0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(ON_L03, vel=60, acc=30, time=0, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        movej(ON_J04, vel=60, acc=30, time=0)
        self.get_logger().info("gripper_put_on motions complete.")
    
    def gripper_put_on_execute(self):
        try:
            from DSR_ROBOT2 import movej, movel, set_velx, set_accx, DR_BASE, DR_TOOL, DR_MV_MOD_ABS, DR_MV_MOD_REL
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        # Set values
        set_velx(100, 20)
        set_accx(1000, 40)

        ON_J01 = [103.83, -0.26, 100.17, 0.0, 80.09, 103.23]
        ON_L02 = [-85.990, 349.210, 336.120, 104.84, 180.0, 104.23]
        ON_L03 = [0.0, -150.0, 0.0, 0.0, 0.0, 0.0]
        ON_J04 = [180.0, 0.0, 90.0, 0.0, 90.0, 90.0]

        # Execute motions
        self.get_logger().info("Executing gripper_put_on motions...")
        movej(ON_J01, vel=60, acc=30, time=0, mod=DR_MV_MOD_ABS)
        time.sleep(1)
        movel(ON_L02, vel=100, acc=1000, time=0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(ON_L03, vel=60, acc=30, time=0, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        movej(ON_J04, vel=60, acc=30, time=0)
        msg = String()
        msg.data = 'close'
        self.gripper_command_publisher.publish(msg)
        self.get_logger().info("gripper_put_on motions complete.")

    def gripper_put_off(self):
        try:
            from DSR_ROBOT2 import movej, movel, set_velx, set_accx, DR_BASE, DR_TOOL, DR_MV_MOD_ABS, DR_MV_MOD_REL
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            return

        # Set values
        set_velx(100, 20)
        set_accx(1000, 40)

        OFF_J01 = [113.26, -20.91, 122.63, -0.46, 76.64, 112.83]
        OFF_L02 = [-85.990, 349.210, 336.120, 104.84, 180.0, 104.23]
        OFF_L03 = [0.0, 0.0, -50.0, 0.0, 0.0, 0.0]
        OFF_J04 = [180.0, 0.0, 90.0, 0.0, 90.0, 90.0]

        # Execute motions
        self.get_logger().info("Executing gripper_put_off motions...")
        movej(OFF_J01, vel=60, acc=30, time=0, mod=DR_MV_MOD_ABS)
        time.sleep(1)
        movel(OFF_L02, vel=100, acc=1000, time=0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(OFF_L03, vel=60, acc=30, time=0, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        movej(OFF_J04, vel=60, acc=30, time=0)
        self.get_logger().info("gripper_put_off motions complete.")

def main(args=None):
    rclpy.init(args=args)
    node = ToolChanger()

    DR_init.__dsr__node = node

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.msg_data == 'ON':
                node.gripper_put_on()
                node.msg_data = 'None'  
            elif node.msg_data == 'OFF':
                node.gripper_put_off()
                node.msg_data = 'None'
            elif node.msg_data == 'OON':
                node.gripper_put_on_execute()
                node.msg_data = 'None'
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
