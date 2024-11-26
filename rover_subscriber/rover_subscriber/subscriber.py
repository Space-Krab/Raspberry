import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        # TODO: Create a subscriber of type Twist, that calls listener_callback
        self.subscription = self.create_subscription(
            Twist,
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info('Subscriber node has been started.')
        self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}

    def listener_callback(self, msg):
        # TODO: Interpret the received commands and log the result using self.get_logger().info()
        msgToDisplay = ""
        if msg.linear.x != 0:
            if msg.linear.z != 0:
                msgToDisplay = "Forbidden move"
            if msg.angular.y != 0:
                msgToDisplay = "Go " + ("Left" if msg.angular.y > 0 else "Right")
            else:
                msgToDisplay = "Go " + ("Forward" if msg.linear.x > 0 else "Backward")
        elif msg.linear.z != 0:
            if msg.angular.y != 0:
                msgToDisplay = "Forbidden move"
            else:
                msgToDisplay = "Slide " + ("Right" if msg.linear.z > 0 else "Left")
        elif msg.angular.y != 0:
            msgToDisplay = "Rotating on itself to the " + ("Left" if msg.angular.y > 0 else "Right")
        if msgToDisplay != "Forbidden move":
            self.position['x'] += msg.linear.x
            self.position['ry'] += msg.angular.y
            self.position['z'] += msg.linear.z
        self.get_logger().info(msgToDisplay)
        self.get_logger().info("New Position: x: {x}, z: {z}, ry: {ry}".format(x = self.position['x'], z = self.position['z'], ry = self.position['ry']))

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()