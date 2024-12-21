import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import cv2
from cv_bridge import CvBridge
import time

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        self.image_subscription = self.create_subscription(
            Image,
            '/techman_image',
            self.image_callback,
            10)
        
        self.capture_subscription = self.create_subscription(
            Empty,
            'capture',
            self.capture_callback,
            10)
        
        self.bridge = CvBridge()
        self.latest_image = None  
        self.image_counter = 0  
        
        self.get_logger().info('Image Capture Node is running. Listening to "capture" topic.')

    def image_callback(self, msg):
        self.latest_image = msg

    def capture_callback(self, msg):
        if self.latest_image is None:
            self.get_logger().warn('No image received yet. Cannot save image.')
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            file_name = f'captured_image_{self.image_counter}_{timestamp}.png'
            cv2.imwrite(file_name, cv_image)
            
            self.image_counter += 1  
            self.get_logger().info(f'Image saved as {file_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
