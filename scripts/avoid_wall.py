#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class AvoidWall(Node):
    pose_x = 0
    pose_y = 0

    def __init__(self):
        super.__init__("avoid_wall")
        self.speed_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, turtle_pose):
        self.pose_x = turtle_pose.x
        self.pose_y = turtle_pose.y

    def timer_callback(self):
        x_ = self.pose_x
        y_ = self.pose_y
        speed = Twist()

        speed.linear.x = 2.0

        if x_ <= 1.0 or x_ >= 10.0 or y_ <= 1.0 or y_ >= 10.0:
            speed.angular.z = 22/7
        else:
            speed.angular.z = 0.0

        self.speed_pub_.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    avoid_wall = AvoidWall()
    rclpy.spin(avoid_wall)
    rclpy.shutdown()

if __name__ == "__main__":
    main()