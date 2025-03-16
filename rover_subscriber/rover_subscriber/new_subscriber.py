import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
from math import sqrt

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
        
        self.rf_direction = 0
        self.rf_speed = 0
        
        self.lf_direction = 0
        self.lf_speed = 0
        
        self.rb_direction = 0
        self.rb_speed = 0
        
        self.lb_speed = 0
        self.lb_direction = 0
        
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info('Subscriber node has been started.')

    def listener_callback(self, msg):
        x = msg.axes[0] # x-axis of left joystick
        y = msg.axes[1] # y-axis of left joystick
        speed = sqrt(x^2 + y^2)
        
        #DIRECTION ASSIGNMENT
        if (x < 0 and y >= -0.2 * x) or (x >= 0 and y >= 0.2 * x):
            self.set_ahead()
        elif ((x < 0 and y <= 0.2 * x) or (x >= 0 and y <= -0.2 * x)):
            self.set_backward()
        elif x < 0:
            self.set_left()
        else:
            self.set_right()
            
        #SPEED ASSIGNMENT
        if ((x < 0 and y >= 0.2 * x and y <= -0.2 * x) #LEFT SECTION 
            or (x >= 0 and y <= 0.2 * x and y >= -0.2 * x) #RIGHT SECTION
            or ((x < 0 and y >= -5 * x) or (x >= 0 and y >= 5 * x)) #FORWARD SECTION
            or ((x < 0 and y <= 5 * x) or (x >= 0 and y <= -5 * x))): #BACKWARD SECTION
            self.set_same_speed(speed)
        else:
            self.set_diff_speed(speed, x)
            
        #DATA SENDING
        self.ser.write(self.lf_speed.to_bytes(1, 'little'))
        self.ser.write(self.lf_direction.to_bytes(1, 'little')) #FRONT LEFT WHEEL
        
        self.ser.write(self.rf_speed.to_bytes(1, 'little'))
        self.ser.write(self.rf_direction.to_bytes(1, 'little')) #FRONT RIGHT WHEEL
        
        self.ser.write(self.rb_speed.to_bytes(1, 'little'))
        self.ser.write(self.rb_direction.to_bytes(1, 'little')) #BACK RIGHT WHEEL
        
        self.ser.write(self.lb_speed.to_bytes(1, 'little'))
        self.ser.write(self.lb_direction.to_bytes(1, 'little')) #BACK LEFT WHEEL
        
        
    def set_ahead(self):
        self.rf_direction = -1
        self.lf_direction = 1
        self.rb_direction = -1
        self.lb_direction = 1
        
    def set_backward(self):
        self.rf_direction = 1
        self.lf_direction = -1
        self.rb_direction = 1
        self.lb_direction = -1
    
    def set_right(self):
        self.rf_direction = 1
        self.lf_direction = 1
        self.rb_direction = 1
        self.lb_direction = 1
        
    def set_left(self):
        self.rf_direction = -1
        self.lf_direction = -1
        self.rb_direction = -1
        self.lb_direction = -1
        
    def set_same_speed(self, speed):
        new_speed = round(speed * 255)
        self.rf_speed = new_speed
        self.lf_speed = new_speed
        self.rb_speed = new_speed
        self.lb_speed = new_speed
        
    def set_diff_speed(self, speed, x):
        diff_speed = round(speed * (1 - abs(x)) * 255)
        new_speed = round(speed * 255)
        if x < 0:
            self.rf_speed = new_speed
            self.rb_speed = new_speed
            self.lf_speed = diff_speed
            self.lb_speed = diff_speed
        else:
            self.rf_speed = diff_speed
            self.rb_speed = diff_speed
            self.lf_speed = new_speed
            self.lb_speed = new_speed
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()