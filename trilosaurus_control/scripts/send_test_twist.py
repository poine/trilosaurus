#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter("frame_id", "odom")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Twist, '/trilosaurus_base_controller/cmd_vel_unstamped', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        outMsg = Twist()
        #outMsg.header = Header()
        #outMsg.header.stamp = self.get_clock().now().to_msg()
        #outMsg.header.frame_id = self.frame_id
        outMsg.angular.z = 0.2;
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(outMsg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
