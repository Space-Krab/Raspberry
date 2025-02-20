import rclpy
from rclpy.node import Node
from custom_msg.msg import Command
import serial

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        # TODO: Create a subscriber of type Twist, that calls listener_callback
        self.subscription = self.create_subscription(
            Command,
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.speed: int = 0
        self.direction: int = 0
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info('Subscriber node has been started.')

    def listener_callback(self, msg):
        # TODO: Interpret the received commands and log the result using self.get_logger().info()
        msgToDisplay = ""
        self.direction = msg.x
        if msg.speed == 1:
            self.speed = self.speed + 16 if self.speed + 16 <= 255 else 255
            msgToDisplay = "Speed Up"
        elif msg.speed == -1:
            self.speed = self.speed - 16 if self.speed - 16 >= 0 else 0
            msgToDisplay = "Slow down"
        if msg.x == 255:
            msgToDisplay = "Stop"
        self.get_logger().info(msgToDisplay)
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