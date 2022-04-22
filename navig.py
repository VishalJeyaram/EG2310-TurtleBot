import rclpy
import RPi.GPIO as GPIO
#from pn532 import *
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
GPIO.setmode(GPIO.BCM)
buttonPin = 12
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Set pin numbering convention
#GPIO.setmode(GPIO.BCM)
# Choose an appropriate pwm channel to be used to control the servo
#servo_pin = 20
# Set the pin as an output
#GPIO.setup(servo_pin, GPIO.OUT)
# Initialise the servo to be controlled by pwm with 50 Hz frequency
#p = GPIO.PWM(servo_pin, 50)
# Set servo to 90 degrees as it's starting position
#p.start(2.5)
 



# constants
#pn532 = PN532_I2C(debug=False, reset=20, req=16)
#ic, ver, rev, support = pn532.get_firmware_version()
#pn532.SAM_configuration()
NFC_state = 0
buttonPress = 0
left_turn_state = 0
right_turn_state = 0
hard_rotatechange = 1.6
soft_rotatechange = 0.6
speedchange = 0.15
state = 1
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def move_straight(self):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    def adjust_left(self):
        global left_turn_state
        twist = Twist()
        twist.linear.x = speedchange
        actual_rotateChange = soft_rotatechange
        left_turn_state += 1
        if (left_turn_state > 40 and (left_turn_state < 60)):
            actual_rotateChange = hard_rotatechange
        twist.angular.z += actual_rotateChange
        self.publisher_.publish(twist)
    def adjust_right(self):
        global left_turn_state
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z -= soft_rotatechange
        self.publisher_.publish(twist)
        left_turn_state -= 1
    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    def hard_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z +=  hard_rotatechange
        self.publisher_.publish(twist)
    def hard_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z -=  hard_rotatechange
        self.publisher_.publish(twist)
        
   
    def scanNFC(self):
        global NFC_state
        print("ENtered scan nfc function")
             # Check if a card is available to read
        uid = pn532.read_passive_target(timeout = 0.00001)
        print('.', end="")
             # Try again if no card is available.
        if uid:
            print('Found card with UID:', [hex(i) for i in uid])
            self.stopbot()
            print("FOUND IT!")
            NFC_state = 1
    
    def button_press(self):
        global buttonPress
        buttonState = GPIO.input(buttonPin)
        if buttonState == False:
            print("pressed")
            buttonPress = 1
          



    def maintain_left_wall_follow(self):
         #if ((self.laser_range[15] < 0.400) and (self.laser_range[0] < 0.350)):
         #       self.hard_right()
         #       print('stationary rotation ')
         global left_turn_state
         #if ((self.laser_range[0] <= 0.400) and (self.laser_range[90] > 0.400) and (self.laser_range[270] > 0.400)):
         #       self.hard_right()
         #       left_turn_state = 0
         #if ((self.laser_range[0] <= 0.400) and (self.laser_range[90] > 0.400)):
         #       self.hard_left()
         #       print("u turn hard_left")
         #       while ((self.laser_range[70] > 0.400) and (self.laser_range[0] > 0.400)):
         #           self.move_straight()
         #           print("move_straight")
         #       print('u-turn')
         #       left_turn_state = 0
         if ((self.laser_range[10] < 0.405) and (self.laser_range[45] <= 0.400) ): 
                self.hard_right()
                print("hard_right")
                left_turn_state = 0
         #elif ((self.laser_range[0] <= 0.500) and (self.laser_range[45] <= 0.400) ): 
         #       self.hard_right()
         #       print("hard_right")
         #elif ((self.laser_range[90]>=0.45) and (self.laser_range[45]>=0.5)):
         #       print("hard_left")
         #       self.hard_left()
         #       time.sleep(1)
         #       self.move_straight()
         #       time.sleep(3)
               
         #elif (((self.laser_range[45] > 0.325) and (self.laser_range[45] < 0.375)) and ((self.laser_range[135] > 0.375) and (self.laser_range[135] < 0.425))):
          #     self.move_straight()
           #    print("move straight")
         elif ((self.laser_range[45] <= 0.250) or (self.laser_range[0] < 0.400)):
               print("too left")
               self.adjust_right()        
         elif ((self.laser_range[45] > 0.250) or (self.laser_range[135]<= 0.300)):
               print("too right")
               self.adjust_left()
         #global NFC_state
         #print("ENtered scan nfc function")
             # Check if a card is available to read
         #uid = pn532.read_passive_target(timeout = 0.000000001)
         #print('.', end="")
             # Try again if no card is available.
         #if uid:
         #      print('Found card with UID:', [hex(i) for i in uid])
         #      self.stopbot()
         #      print("FOUND IT!")
         #      NFC_state = 1

         #else:
         #      print("extra")
         #      self.move_straight()
    
    #def ChangeAngle(angle):
    #     p.ChangeDutyCycle(2+(float(angle)/18))
    #     time.sleep(0.25)
 
    #def load_next_ball(self):
    #     try:
    #          for i in range(1):
    #               self.ChangeAngle(30)
    #               self.ChangeAngle(0)
    
    #     except KeyboardInterrupt:
    #          p.stop()
    #          GPIO.cleanup()
       

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw 
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)



    def mover(self):
        global NFC_state
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            #self.pick_direction()
            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    #lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    #self.get_logger().info('Distances: %s' % str(lri))
                    if ((NFC_state == 1) and (buttonPress == 0)):
                        self.button_press()
                        self.stopbot()
                    elif (NFC_state == 0):
                        #self.scanNFC()
                        self.maintain_left_wall_follow()
                    else:
                        self.maintain_left_wall_follow()
                    #print(self.laser_range[45])
                    #self.hard_right()
                    #while (True):
                    #    self.stopbot()
                   # if the list is not empty
                   #if(len(lri[0])>0):
                        # stop moving
                   #     self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                   #     self.pick_direction()

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


