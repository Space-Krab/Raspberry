import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import serial
from math import sqrt, pi

WHEEL_DIAMETER_CM = 11.4
TICKS_PER_REV = 11
WHEEL_BASE_CM = 20.0

QOS_PROFILE_JOY = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=1
)

QOS_PROFILE_ODOM = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    depth=10
)

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        
        self.subscription = self.create_subscription(Joy, 
                                                     '/cmd_vel', 
                                                     self.listener_callback,
                                                     QOS_PROFILE_JOY)
        
        self.subscription2 = self.create_subscription(Joy, 
                                                      '/joy', 
                                                      self.mode_listener_callback, 
                                                      QOS_PROFILE_JOY)
        
        self.publisher = self.create_publisher(String, '/odom', QOS_PROFILE_ODOM)
        
        self.autonomous_mode = False
        self.prev_triangle = 0
        self.prev_fl = 0
        self.prev_fr = 0
        self.curr_rotation = 0
        self.curr_distance = 0
        self.curr_command = ""
        self.timer = self.create_timer(0.2, self.read_serial)
        
        self.mode_timer = self.create_timer(0.02, self.mode_change)
        self.prev_mode_msg = None
        
        self.rf_direction = 0 #FRONT RIGHT WHEEL
        self.rf_speed = 0
        
        self.lf_direction = 0 #FRONT LEFT WHEEL
        self.lf_speed = 0
        
        self.rb_direction = 0 #BACK RIGHT WHEEL
        self.rb_speed = 0
        
        self.lb_speed = 0 #BACK LEFT WHEEL
        self.lb_direction = 0
        
        self.prev_motor_state = (self.lf_speed, self.lf_direction, self.rf_speed, self.rf_direction,
                 self.rb_speed, self.rb_direction, self.lb_speed, self.lb_direction)
        
        #Arduino master connected port
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.get_logger().info('Subscriber node has been started.')
        
        
    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()  # e.g., "FL:120,FR:122"
            self.get_logger().info(line)
            #self.ser.reset_input_buffer()
            #self.ser.readline()
            if not line.startswith("FL"):
                return
            fl_ticks = int(line.split(",")[0].split(":")[1])
            fr_ticks = int(line.split(",")[1].split(":")[1])

            # Compute deltas
            delta_fl = fl_ticks - self.prev_fl
            delta_fr = fr_ticks - self.prev_fr
            self.prev_fl = fl_ticks
            self.prev_fr = fr_ticks

            # Compute distances
            dist_fl = (delta_fl / TICKS_PER_REV) * pi * WHEEL_DIAMETER_CM
            dist_fr = (delta_fr / TICKS_PER_REV) * pi * WHEEL_DIAMETER_CM
            distance_cm = (dist_fl + dist_fr) / 2.0

            # Compute rotation (angle in degrees)
            angle_deg = ((dist_fr - dist_fl) / WHEEL_BASE_CM) * 180 / pi
            
            if self.autonomous_mode:
                self.curr_distance += distance_cm
                self.curr_rotation += angle_deg
                self.get_logger().info(f"Curr: distance = {self.curr_distance:.2f} cm, Curr rotation = {self.curr_rotation:.2f}°")
                self.get_logger().info(self.curr_command)
                if self.curr_command == "up":
                    if self.curr_distance >= 30:
                        self.finish_step()
                elif self.curr_command == "down":
                    if self.curr_distance <= -30:
                        self.finish_step()
                elif self.curr_command == "left":
                    if self.curr_rotation >= 90:
                        self.finish_step()
                elif self.curr_command == "right":
                    if self.curr_rotation <= -90:
                        self.finish_step()
                
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")
            
    def mode_listener_callback(self, msg):
        self.prev_mode_msg = msg
        
    def mode_change(self):
        if self.prev_mode_msg is None:
            return
        
        triangle = self.prev_mode_msg.buttons[2]

        # Toggle mode
        if triangle == 1 and self.prev_triangle == 0:
            self.autonomous_mode = not self.autonomous_mode
            mode = "autonome" if self.autonomous_mode else "manuel"
            self.get_logger().info(f"Mode changé : {mode}")
        
        self.prev_triangle = triangle

    def listener_callback(self, msg):
        if self.autonomous_mode:
            if msg.axes[1] == 1.0:
                self.curr_command = "up"
            elif msg.axes[1] == -1.0:
                self.curr_command = "down"
            elif msg.axes[0] == 1.0:
                self.curr_command = "left"
            elif msg.axes[0] == -1.0:
                self.curr_command = "right"
        
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
        self.send_data()
        
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
        
    def finish_step(self):
        msg = String()
        msg.data = "OK"
        self.publisher.publish(msg)
        self.get_logger().info(f"Finishing step {self.curr_command}")
        
        self.rf_direction = 0 #FRONT RIGHT WHEEL
        self.rf_speed = 0
        
        self.lf_direction = 0 #FRONT LEFT WHEEL
        self.lf_speed = 0
        
        self.rb_direction = 0 #BACK RIGHT WHEEL
        self.rb_speed = 0
        
        self.lb_speed = 0 #BACK LEFT WHEEL
        self.lb_direction = 0
        
        self.send_data()
        
        self.curr_rotation = 0
        self.curr_distance = 0
        
    def send_data(self):
        new_state = (self.lf_speed, self.lf_direction, self.rf_speed, self.rf_direction,
                 self.rb_speed, self.rb_direction, self.lb_speed, self.lb_direction)
        if new_state == self.prev_motor_state:
            return
        self.prev_motor_state = new_state
        
        self.get_logger().info("SENT INFO")
        self.get_logger().info(str(self.lf_speed))
        self.get_logger().info(str(self.lf_direction))
        self.ser.write(self.lf_speed.to_bytes(1, 'little'))
        self.ser.write(self.lf_direction.to_bytes(1, 'little')) #FRONT LEFT WHEEL

        self.get_logger().info(str(self.rf_speed))
        self.get_logger().info(str(self.rf_direction))
        self.ser.write(self.rf_speed.to_bytes(1, 'little'))
        self.ser.write(self.rf_direction.to_bytes(1, 'little')) #FRONT RIGHT WHEEL
        
        self.get_logger().info(str(self.rb_speed))
        self.get_logger().info(str(self.rb_direction))
        self.ser.write(self.rb_speed.to_bytes(1, 'little'))
        self.ser.write(self.rb_direction.to_bytes(1, 'little')) #BACK RIGHT WHEEL
        
        self.get_logger().info(str(self.lb_speed))
        self.get_logger().info(str(self.lb_direction))
        self.ser.write(self.lb_speed.to_bytes(1, 'little'))
        self.ser.write(self.lb_direction.to_bytes(1, 'little')) #BACK LEFT WHEEL

        self.get_logger().info(self.ser.readline().decode().strip())
        self.get_logger().info(self.ser.readline().decode().strip())
        self.get_logger().info(self.ser.readline().decode().strip())
        self.get_logger().info(self.ser.readline().decode().strip())

        
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
        # factor (1 - abs(x)) bounded by 0 and 1, meaning they will 
        # be slowed the more sharply we want to turn, and an additional
        # factor to slow even more the speed of the wheels
        diff_weighted_speed = round(speed * 255 * (1 - abs(x)) * 0.15)
        
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
