import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        # TODO: Create a subscriber of type Twist, that calls listener_callback
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        self.subscription
        self.speed: int = 0
        self.direction: int = 0
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info('Subscriber node has been started.')

    def listener_callback(self, msg):
        left_vertical = msg.axes[1]
        self.direction = 0 if left_vertical < 0 else 1
        left_vertical_abs = abs(left_vertical) - 0.3
        self.speed = (left_vertical_abs * 255) / 0.7
        self.ser.write(self.speed.to_bytes(1, 'little'))
        self.ser.write(self.direction.to_bytes(1, 'little'))
        self.get_logger().info(f'Computer : {self.speed.to_bytes(1, 'little')} {self.direction}')
        self.get_logger().info(self.ser.readline().decode('utf-8').rstrip())
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()