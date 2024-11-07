import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescImuStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt
from queue import Queue

class DriftingDetector(Node):
    def __init__(self):
        super().__init__('drifting_detector')
        
        self.wheelbase = 0.32  # 32 cm in meters
        self.tirewidth = 0.04445
        self.mass = 3.333
        self.gravity = 9.81
        self.force_of_gravity = self.mass * self.gravity
        self.ackermann_subscriber = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.ackermann_callback,
            10)
        
        self.imu_subscriber = self.create_subscription(
            VescImuStamped,
            '/sensors/imu',
            self.imu_callback,
            10)
        
        self.drifting_publisher = self.create_publisher(Bool, 'is_drifting', 10)
        self.angular_vel_publisher = self.create_publisher(String, 'angular_vel', 10)
        self.linear_vel_publisher = self.create_publisher(String, 'linear_vel', 10)
        self.friction_publisher = self.create_publisher(Float64, 'friction_mu', 10)

        self.current_velocity = 0.0
        self.linear_velocity = 0.0
        self.current_steering_angle = 0.0
        self.current_angular_velocity = 0.0
        self.historical_acc = []
        self.time_stamps = []

    def ackermann_callback(self, msg):
        self.current_velocity = msg.drive.speed
        self.current_steering_angle = msg.drive.steering_angle

    def imu_callback(self, msg):
        self.current_angular_velocity = msg.imu.angular_velocity.z
        self.linear_acceleration = msg.imu.linear_acceleration.y
        if len(self.historical_acc) < 20:
        	self.historical_acc.append(self.linear_acceleration)
        	self.time_stamps.append(msg.header.stamp.sec + (msg.header.stamp.nanosec * 10**-9))
        else:
        	self.time_stamps = self.time_stamps[1:]
        	self.time_stamps.append(msg.header.stamp.sec + (msg.header.stamp.nanosec * 10**-9))
        	self.historical_acc = self.historical_acc[1:]
        	self.historical_acc.append(self.linear_acceleration)
        self.check_drifting()
        self.calculate_friction()

    def check_drifting(self):
        # Calculate theoretical angular velocity
        turning_radius = 0.0
        if self.current_steering_angle == 0:
            turning_radius = 0.0
            times = self.time_stamps
            linear_acc = self.historical_acc
            print('lin_acc:', linear_acc)
            print('timestamps:', times)
            self.linear_velocity = integrate.cumtrapz(linear_acc, times, initial=0) + self.linear_velocity
            print('lin_vel:', self.linear_velocity)
            self.linear_velocity = self.linear_velocity[-1]
            linear_msg = Float64()
            #linear_msg.data = linear_velocity
            #self.linear_vel_publisher.publish(linear_msg)	
          #  if linear_velocity != self.current_velocity: # add an error bound here for difference in caluclated linear velocity and speed read by the odom
              #  drifting_msg = Bool()
              #  drifting_msg.data = is_drifting
              #  self.drifting_publisher.publish(drifting_msg)
        else:
        	turning_radius = (self.wheelbase/(np.sin(self.current_steering_angle)) + 0.5*(self.tirewidth))
        	if self.current_velocity > 0:  # Prevent division by zero
        		theoretical_angular_velocity = self.current_velocity * turning_radius
        		is_drifting = 20 < (abs(self.current_angular_velocity) - abs(theoretical_angular_velocity))
        		drifting_msg = Bool()
        		drifting_msg.data = bool(is_drifting)
        		self.drifting_publisher.publish(drifting_msg)
    
    def angular_velocity(self):
    	# Calcualte and publish angular velocity
        return

    def calculate_friction(self):
    	turning_radius = 0.0
    	if self.current_steering_angle == 0:
        	turning_radius = 0
        	times = self.time_stamps
        	linear_acc = self.historical_acc
        	self.linear_velocity = integrate.cumtrapz(linear_acc, times, initial=0)[-1] + self.linear_velocity
		# TODO: logic here to calculate coefficient of friction while driving in a straight line
    	else:
    		turning_radius = (self.wheelbase/(np.sin(self.current_steering_angle)) + 0.5*(self.tirewidth))
    	if turning_radius > 0:
    		force_of_friction = self.mass * ((self.current_velocity * self.current_velocity) / turning_radius)
    		mu = force_of_friction / self.force_of_gravity
		
    		friction_val = Float64()
    		friction_val.data = mu
    		self.friction_publisher.publish(friction_val)

def main(args=None):
    rclpy.init(args=args)
    drifting_detector = DriftingDetector()
    rclpy.spin(drifting_detector)
    drifting_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

