#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

def set_out_of_range_to_zero(values):
    # Set values out of the range [-0.001, 0.001] to zero
    values[(values < -0.001) | (values > 0.001)] = 0
    return values

class OmniRobotController(Node):
    def __init__(self):
        super().__init__('omnirobot_controller')

        # Definition of the robot's physical parameters
        self.L = 0.1697  # Distance from the robot's center to each wheel (in meters)
        self.Rw = 0.05  # Wheel radius (in meters)

        # Creation of the publisher for wheel speeds
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Creation of the subscriber for the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialization of wheel speeds
        self.wheel_vel = np.array([0.0, 0.0, 0.0, 0.0], float)

    def cmd_vel_callback(self, msg):
        """
        Callback executed when data are received on /cmd_vel.
        Calculates the wheel speeds based on the translation and rotation commands.
        """
        # Extract linear and angular speeds from the Twist message
        vel_x = msg.linear.x  # Linear speed in x
        vel_y = msg.linear.y  # Linear speed in y
        vel_w = msg.angular.z  # Angular speed around z

        # Calculation of wheel speeds via inverse kinematics
        self.wheel_vel[0] = (vel_x * math.sin(math.pi / 4)            + vel_y * math.cos(math.pi / 4)            + self.L * vel_w) / self.Rw
        self.wheel_vel[1] = (vel_x * math.sin(math.pi / 4 + math.pi / 2) + vel_y * math.cos(math.pi / 4 + math.pi / 2) + self.L * vel_w) / self.Rw
        self.wheel_vel[2] = (vel_x * math.sin(math.pi / 4 - math.pi)   + vel_y * math.cos(math.pi / 4 - math.pi)   + self.L * vel_w) / self.Rw
        self.wheel_vel[3] = (vel_x * math.sin(math.pi / 4 - math.pi / 2) + vel_y * math.cos(math.pi / 4 - math.pi / 2) + self.L * vel_w) / self.Rw

        self.wheel_vel = set_out_of_range_to_zero(self.wheel_vel)

        # Publish the wheel speeds
        array_for_publish = Float64MultiArray(data=self.wheel_vel)
        self.publisher_.publish(array_for_publish)

        self.get_logger().info(f'Wheel velocities: {self.wheel_vel}')


def main(args=None):
    rclpy.init(args=args)

    # Launch the controller
    omnirobot_controller = OmniRobotController()

    rclpy.spin(omnirobot_controller)
    

if __name__ == '__main__':
    main()
