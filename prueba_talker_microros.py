#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MicroRosPublisher(Node):

  def __init__(self):
    super().__init__('microros_publisher')
    self.publisher_ = self.create_publisher(String, '/motores/cmd_vel', 10)
    hilo = threading.Thread(target=self.timer_callback)
    hilo.start()
    self.i = 0

  def timer_callback(self):
    while(True):
      msg = String()
      self.get_logger().info("Waiting command...")
      command = input()
      msg.data = command
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg)
    """msg = String()
    msg.data = str(self.i)
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    if self.i < 1000:
      self.i += 10
    if self.i == 1000:
      self.i = 0"""


def main(args=None):
  rclpy.init(args=args)

  microros_publisher = MicroRosPublisher()

  rclpy.spin(microros_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  microros_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


