import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class subscriber (Node):
    def __init__(self):
        super().__init__("Subscriber_Node")
        self.get_logger().info("The node is working!")
        self.subscribeLaser_ = self.create_subscription(LaserScan,"/scan",self.scan_callback,10)
        self.subscribeOdometry_ = self.create_subscription(Odometry,"/ego_racecar/odom",self.odom_callback,10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped,"/drive",10)
        #self.timer = self.create_timer(100,self.pub_brake)

        self.min_angle = 0.0
        #self.max_angle = 0.0
        self.angle_increment = 0.0
        #self.time_increment = 0.0
        #self.scan_time = 0.0
        #self.range_min = 0.0
        #self.range_max = 0.0
        #self.ranges = []
        #self.intensities = []
        self.iTTC_threshold =0.8

    
        self.odom_linear_velocity = 0.0


    def scan_callback(self, msg: LaserScan):
        self.min_angle = msg.angle_min * (180.0 / np.pi)
        #self.max_angle = msg.angle_max * (180.0 / np.pi)
        self.angle_increment = msg.angle_increment * (180.0 / np.pi)
        #self.time_increment = msg.time_increment
        #self.scan_time = msg.scan_time
        #self.range_min = msg.range_min
        #self.range_max = msg.range_max
        ranges = np.array(msg.ranges) ##chat gpt
        range_rate =  self.odom_linear_velocity
        iTTC = ranges / range_rate
        iTTC[np.isinf(iTTC) | np.isnan(iTTC)] = self.iTTC_threshold + 1
        # Decide whether to brake
        #self.iTTC_threshold = self.calculate_threshold(self.odom_linear_velocity)   
        #self.get_logger().info(f"threshold = {self.iTTC_threshold}")     
        if np.min(iTTC) < self.iTTC_threshold:
            self.get_logger().info("Minimum iTTC: {}".format(np.min(iTTC)))
            self.pub_brake()

    def calculate_threshold(self,velocity):
        if velocity < 1:
            self.get_logger().info(f"velocity = {velocity}")
            return 1
        elif velocity > 1 and velocity < 1.5:
            #self.iTTC_threshold = 0.8
            self.get_logger().info(f"velocity = {velocity}")
            return 0.8
        elif velocity > 1.5 and velocity < 2.5 :
            #self.iTTC_threshold = 0.6
            self.get_logger().info(f"velocity = {velocity}")
            return 0.6
        else:
            return 0.1

    def odom_callback(self, msg: Odometry):
        self.odom_linear_velocity = msg.twist.twist.linear.x
        #self.get_logger().info("Linear velocity: {}".format(self.odom_linear_velocity))

    def pub_brake(self):
        #relative_velocity = self.odom_linear_velocity
            
            brake = AckermannDriveStamped()
            brake.drive.speed = 0.0
            self.publisher_.publish(brake)
            self.get_logger().info("Emergency brakes applied")

                


def main(args=None):
    rclpy.init(args=args)
    node = subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
