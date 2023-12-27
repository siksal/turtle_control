#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from turtlesim.msg import Pose
import numpy as np

x1 = x2 = y1 = y2 = 0.0

def pose_cb(turtle_pose = Pose()):
    global x1, y1
    x1 = turtle_pose.x
    y1 = turtle_pose.y

def eds_cb(request=AddTwoInts.Request(), response=AddTwoInts.Response()):
    global x2, y2
    x2 = request.a
    y2 = request.b

    print("Incoming Coordinates: (%d, %d), (%d, %d)" % (x1, y1, x2, y2))
    response.sum = int(np.sqrt(np.square(x2 - x1) + np.square(y2 -y1)))
    print("Sending back response: [%d]" % response.sum)
    return response

def main(args=None):
    rclpy.init(args=args)
    node = Node("eds")
    pose_sub = node.create_subscription(Pose, "/turtle1/pose", pose_cb, 10)
    server = node.create_service(AddTwoInts, "dist_two_points", eds_cb)
    node.get_logger().info("Ready to get distance between two points")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()