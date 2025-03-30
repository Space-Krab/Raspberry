import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
from math import sqrt

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber') #node name and instanciation
        
        self.subscription = self.create_subscription(
            Joy, #receive Joy messages
            '/joy', #topic name to subscribe to
            self.listener_callback, #method to call when receiving a message
            10)
        
        self.rf_direction = 0 #FRONT RIGHT WHEEL
        self.rf_speed = 0
        
        self.lf_direction = 0 #FRONT LEFT WHEEL
        self.lf_speed = 0
        
        self.rb_direction = 0 #BACK RIGHT WHEEL
        self.rb_speed = 0
        
        self.lb_speed = 0 #BACK LEFT WHEEL
        self.lb_direction = 0
        
        #Arduino master connected port
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info('Subscriber node has been started.')

    def listener_callback(self, msg):
        x = msg.axes[0] # x-axis of left joystick
        
        #SAFETY BOUND FOR X
        if x < -1:
            x = -1
        elif x > 1:
            x = 1
            
        y = msg.axes[1] # y-axis of left joystick
        
        #SAFETY BOUND FOR Y
        if y < -1:
            y = -1
        elif y > 1:
            y = 1
            
        speed = sqrt(x**2 + y**2) #euclidean distance to the center
        
        #SAFETY BOUND FOR SPEED
        if speed > 1:
            speed = 1
        
        #DIRECTION ASSIGNMENT
        if (x < 0 and y >= -0.2 * x) or (x >= 0 and y >= 0.2 * x):
            self.set_ahead()
        elif ((x < 0 and y <= 0.2 * x) or (x >= 0 and y <= -0.2 * x)):
            self.set_backward()
        elif x < 0:
            self.set_right()
        else:
            self.set_left()
            
        #SPEED ASSIGNMENT
        if ((x < 0 and y >= 0.2 * x and y <= -0.2 * x) #RIGHT SECTION 
            or (x >= 0 and y <= 0.2 * x and y >= -0.2 * x) #LEFT SECTION
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
        
        #LOGS PRINTING
        if (x < 0 and y >= 0.2 * x and y <= -0.2 * x):
            self.get_logger().info("ROTATE RIGHT")
        elif (x >= 0 and y <= 0.2 * x and y >= -0.2 * x):
            self.get_logger().info("ROTATE LEFT")
        elif ((x < 0 and y >= -5 * x) or (x >= 0 and y >= 5 * x)):
            self.get_logger().info("GO FORWARD")
        elif ((x < 0 and y <= 5 * x) or (x >= 0 and y <= -5 * x)):
            self.get_logger().info("GO BACKWARD")
        elif (x < 0 and y <= -5 * x and y >= -0.2 * x):
            self.get_logger().info("DIFFERENTIAL FRONT RIGHT")
        elif (x < 0 and y <= 0.2 * x and y >= 5 * x):
            self.get_logger().info("DIFFERENTIAL BACK RIGHT")
        elif (x > 0 and y >= 0.2 * x and y <= 5 * x):
            self.get_logger().info("DIFFERENTIAL FRONT LEFT")
        elif (x > 0 and y <= -0.2 * x and y >= -5 * x):
            self.get_logger().info("DIFFERENTIAL BACK LEFT")
        else:
            self.get_logger().info("Not moving")
        
        self.get_logger().info(f"SPEED: {round(speed * 255)}")
        
        
    def set_ahead(self):
        self.rf_direction = 255
        self.lf_direction = 1
        self.rb_direction = 255
        self.lb_direction = 1
        
    def set_backward(self):
        self.rf_direction = 1
        self.lf_direction = 255
        self.rb_direction = 1
        self.lb_direction = 255
    
    def set_right(self):
        self.rf_direction = 1
        self.lf_direction = 1
        self.rb_direction = 1
        self.lb_direction = 1
        
    def set_left(self):
        self.rf_direction = 255
        self.lf_direction = 255
        self.rb_direction = 255
        self.lb_direction = 255
        
    def set_same_speed(self, speed):
        """
        Sets the same speed to all 4 wheels 

        Args:
            speed (float): speed value between 0 and 1
        """
        weighted_speed = round(speed * 255)
        
        self.rf_speed = weighted_speed
        self.lf_speed = weighted_speed
        self.rb_speed = weighted_speed
        self.lb_speed = weighted_speed
        
    def set_diff_speed(self, speed, x):
        """
        Sets differential speed to the wheels depending on 
        which side we want to turn to

        Args:
            speed (float): absolute speed value to be used for 
                            differential computations
            x (float): horizontal value of the left joystick
        """
        weighted_speed = round(speed * 255) #max speed is 255
        
        #the inside the turn wheels' speeds are multiplied by a 
        # factor (1 - 0.5 * abs(x)) bounded by 0 and 1, meaning they will 
        # be slowed the more sharply we want to turn
        diff_weighted_speed = round(speed * 255 * (1 - 0.5 * abs(x)))
        
        if x > 0: #turn left
            self.rf_speed = weighted_speed
            self.rb_speed = weighted_speed
            self.lf_speed = diff_weighted_speed
            self.lb_speed = diff_weighted_speed
        else: #turn right
            self.rf_speed = diff_weighted_speed
            self.rb_speed = diff_weighted_speed
            self.lf_speed = weighted_speed
            self.lb_speed = weighted_speed
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
