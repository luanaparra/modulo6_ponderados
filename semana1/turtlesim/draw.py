#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Draw(Node):

    def __init__(self):
        super().__init__("draw")
        self.pub_vel_ = self.create_publisher(Twist, "/turtle/cmd_vel", 10)
        self.timer_ = self.create_timer(2.0, self.send_pub_vel)
        self.part = 1
        self.msg = Twist()
        self.get_logger().info("Drawing...")

    def movement1(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 0.0
        self.pub_vel_.publish(self.msg)

    def movement2(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 2.0
        self.pub_vel_.publish(self.msg)

    def movement3(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 4.0
        self.pub_vel_.publish(self.msg)

    def movement4(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 6.0
        self.pub_vel_.publish(self.msg)

    def movement5(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 8.0
        self.pub_vel_.publish(self.msg)

    def movement6(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 10.0
        self.pub_vel_.publish(self.msg)

    def movement7(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 12.0
        self.pub_vel_.publish(self.msg)

    def movement8(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 14.0
        self.pub_vel_.publish(self.msg)

    def movement9(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 16.0
        self.pub_vel_.publish(self.msg)

    def movement10(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 18.0
        self.pub_vel_.publish(self.msg)

    def movement11(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 20.0
        self.pub_vel_.publish(self.msg)

    def movement12(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 18.0
        self.pub_vel_.publish(self.msg)

    def movement13(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 16.0
        self.pub_vel_.publish(self.msg)

    def movement14(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 14.0
        self.pub_vel_.publish(self.msg)

    def movement15(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 12.0
        self.pub_vel_.publish(self.msg)

    def movement16(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 10.0
        self.pub_vel_.publish(self.msg)

    def movement17(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 8.0
        self.pub_vel_.publish(self.msg)

    def movement18(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 6.0
        self.pub_vel_.publish(self.msg)

    def movement19(self):
        self.msg.linear.x = 0.5
        self.msg.angular.z = 4.0
        self.pub_vel_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = Draw()
    rclpy.spin(node)
    rclpy.shutdown()
