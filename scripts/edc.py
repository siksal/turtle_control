#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Client(Node):
    def __init__(self):
        super().__init__("edc")
        self.client = self.create_client(AddTwoInts, "dist_two_points")
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().info("Interrupted while waiting for the service. Exiting")
            else:
                self.get_logger().info("Service not available, waiting again...")

        self.request = AddTwoInts.Request()

    def send_request(self):
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.result = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Warning: Input two coordinates")
        sys.exit()
    dist_client = Client()
    dist_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(dist_client)
        if dist_client.result.done():
            try:
                response = dist_client.result.result()
            except Exception as e:
                dist_client.get_logger().info("Failed to call service %r" % (e,))
            else:
                dist_client.get_logger().info("Distance between pose: %d" % response.sum)
            break
    dist_client.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()