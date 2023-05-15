#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from time import sleep
from tf_transformations import euler_from_quaternion
import math

MAX_DIFF = 0.1

global posicoes_rota 
posicoes_rota = [
    [2.5, 2.0],
    [3.0, 1.5],
    [2.0, 2.5],
    [2.0, 3.5],
    [1.0, 3.0]
]

class TurtleController(Node):

    def __init__(self, control_period=0.05):
        super().__init__('turtle_controller')
        self.x_array = 0

        self.odom = Odometry()
        self.actual_pose = self.odom.pose.pose
        self.actual_pose_x = self.actual_pose.position.x
        self.actual_pose_y = self.actual_pose.position.y

        self.actual_pose_x = posicoes_rota[self.x_array][0]
        self.actual_pose_y = posicoes_rota[self.x_array][1]

        self.setpoint_odom = Odometry()
        self.setpoint = self.setpoint_odom.pose.pose
        self.setpoint_x = self.setpoint.position.x
        self.setpoint_y = self.setpoint.position.y

        self.setpoint_x = posicoes_rota[self.x_array+1][0]
        self.setpoint_y = posicoes_rota[self.x_array+1][1]

        self.control_period = control_period
        self.control_timer = self.create_timer(self.control_period, self.control_callback)

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.odom = msg
        self.actual_pose = self.odom.pose.pose

    def control_callback(self):
        self.actual_pose_x = self.actual_pose.position.x
        self.actual_pose_y = self.actual_pose.position.y

        x_error = self.setpoint_x - self.actual_pose_x
        y_error = self.setpoint_y - self.actual_pose_y

        dist_error = math.sqrt(x_error ** 2 + y_error ** 2)

        if dist_error < MAX_DIFF:
            self.x_array += 1
            if self.x_array == len(posicoes_rota) - 1:
                self.x_array = 0

            self.setpoint_x = posicoes_rota[self.x_array + 1][0]
            self.setpoint_y = posicoes_rota[self.x_array + 1][1]

        yaw = euler_from_quaternion([self.actual_pose.orientation.x, self.actual_pose.orientation.y,
                                      self.actual_pose.orientation.z, self.actual_pose.orientation.w])[2]

        setpoint_yaw = math.atan2(y_error, x_error)

        error_yaw = setpoint_yaw - yaw

        if error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        elif error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        k_p = 1.0
        angular_velocity = k_p * error_yaw

        linear_velocity = 0.2

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity

        self.vel_publisher.publish(vel_msg)

def main(args=None):
    rcl.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





