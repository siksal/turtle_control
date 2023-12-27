#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtle_control.msg import MessageCounter

class Counter(Node):
    msg_counter = MessageCounter()
    msg_counter.name = "Robot"
    msg_counter.count = 0
    msg_counter.state = True

    def __init__(self):
        super().__init__("message_counter")
        self.publisher_ = self.create_publisher(MessageCounter, "count_number", 10)
        self.timer_ = self.create_timer(1.0, self.pub_count)

    def pub_count(self):
        if self.msg_counter.count == 10:
            self.msg_counter.count = 0
            self.msg_counter.state = not self.msg_counter.state
            print("Hey " + str(self.msg_counter.name) + "! Starting again.")
            print("State changed")

        print("Counter is" + str(self.msg_counter.count))
        print("State is " + str(self.msg_counter.state))

        self.publisher_.publish(self.msg_counter)
        self.msg_counter.count += 1

def main(args=None):
    rclpy.init(args=args)
    counter = Counter()
    rclpy.spin(counter)
    rclpy.shutdown()

if __name__ == "__main__":
    main()