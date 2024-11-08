import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter("input_delay_topic", "input_delay_topic")
        # 获取输入话题名称
        input_topic = self.get_parameter('input_delay_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Float64MultiArray, input_topic, 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.i/10, 2*self.i/10]  # 生成一个随机的5元素数组
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i = self.i + 1


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
