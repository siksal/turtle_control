#!/usr/bin/env python3

from urllib import response
import rclpy
from rclpy.node import Node
from turtle_control.srv import Stop

class Client(Node):
    def __init__(self):
        super().__init__("stop_robot_client")
        self.client = self.create_client(Stop, "stop_robot_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().info("Interrupted while waiting for the service. Exiting")
            else:
                self.get_logger().info("Service not available, waiting again ...")
        self.request = Stop.Request()

    def send_request(self):
        self.request.front_wheel = 0
        self.request.rear_wheel = 0
        self.result = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    stop_client = Client()
    stop_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(stop_client)
        if stop_client.result.done():
            try:
                response = stop_client.result.result()
            except Exception as e:
                stop_client.get_logger().info("Failed to call service %r" % (e,))
            else:
                stop_client.get_logger().info("Stop Robot: %s" % response.stop_robot)
            break

    stop_client.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()