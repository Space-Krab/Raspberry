#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    
    def _init_(self):
        super()._init_('compressed_image_subscriber')
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',
            self.image_callback,
            1  # Small queue to drop old frames (reduce latency)
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("Subscribed to compressed images")
        
    def image_callback(self, msg):
        try:
            # Convert compressed data to OpenCV (no need for cv_bridge)
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Display with timestamp (to check latency)
            cv2.putText(cv_image, f"ROS Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Live Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    subscriber = CompressedImageSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()