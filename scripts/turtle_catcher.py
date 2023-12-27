#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from math import pow, atan2, sqrt

## you cannot use while loop inside a timer callback. Use if instead.

class TurtleCatcher(Node):

    def __init__(self):
        super().__init__("catch_turtle")

        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.update_pose, 10)

        self.timer = self.create_timer(0.01, self.catch_turtle)

        self.pose = Pose()
        self.vel_msg = Twist()
        self.goal_pose = Pose()
        self.stop = False
        self.spawn_first = True
        self.count = 0

    def spawn_turtle(self, goal):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Spawn...")

        request = Spawn.Request()
        request.x = goal.x
        request.y = goal.y
        request.theta = goal.theta
        request.name = "turtle_" + str(self.count)

        future = client.call_async(request)

    def kill_turtle(self):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Kill...")

        request = Kill.Request()
        request.name = "turtle_" + str(self.count)

        future = client.call_async(request)

    def update_pose(self, data):
        self.pose.x = round(data.x, 4)
        self.pose.y = round(data.y, 4)
        self.pose.theta = round(data.theta, 4)

    def euclidean_distance(self):
        return sqrt(pow((self.goal_pose.x - self.pose.x), 2) +
                    pow((self.goal_pose.y - self.pose.y), 2))

    def linear_vel(self, constant=1.5):
        return constant * self.euclidean_distance()

    def steering_angle(self):
        return atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)

    def angular_vel(self, constant=6):
        return constant * (self.steering_angle() - self.pose.theta)

    def catch_turtle(self, distance_tolerance=0.6):
        # self.goal_pose = Pose()
        # vel_msg = Twist()

        if(self.spawn_first == True):
            self.count = 1
            self.goal_pose.x = float(random.randrange(1, 10))
            self.goal_pose.y = float(random.randrange(1, 10))

            self.spawn_turtle(self.goal_pose)
            self.spawn_first = False

        if(self.euclidean_distance() > distance_tolerance and self.spawn_first == False):
            self.vel_msg.linear.x = self.linear_vel()
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0

            self.vel_msg.angular.x = 0.0
            self.vel_msg.angular.y = 0.0
            self.vel_msg.angular.z = self.angular_vel()

            self.velocity_publisher.publish(self.vel_msg)

        if(self.euclidean_distance() < distance_tolerance and self.spawn_first == False):
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.vel_msg)
            self.stop = True

        if(self.stop == True):
            self.kill_turtle()

            self.goal_pose.x = float(random.randrange(1, 10))
            self.goal_pose.y = float(random.randrange(1, 10))

            self.count += 1

            self.spawn_turtle(self.goal_pose)
            self.stop = False


def main(args=None):
    rclpy.init(args=args)
    turtle_catcher = TurtleCatcher()
    rclpy.spin(turtle_catcher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()