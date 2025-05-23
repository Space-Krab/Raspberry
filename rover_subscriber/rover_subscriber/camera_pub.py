#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CompressedImagePublisher(Node):
    
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/compressed', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 FPS
        self.cap = cv2.VideoCapture(0)  # Pi Camera or USB camera
        self.get_logger().info("Publishing compressed images...")
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Compress to JPEG (adjust quality [0-100], lower = smaller but worse quality)
            _, compressed_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = compressed_data.tobytes()
            self.publisher_.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = CompressedImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()