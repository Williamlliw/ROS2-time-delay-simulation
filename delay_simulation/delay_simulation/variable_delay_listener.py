from rclpy.node import Node
import rclpy
import random
import math

from std_msgs.msg import Float64MultiArray


# 2. 创建节点参数服务节点
class python_delayed_listener(Node):
    def __init__(self):
        # 2.1 构造函数
        super().__init__("delay_listener", allow_undeclared_parameters=True)
        # 定义延时参数：延时类型delay_type，平均延时average_delay_time，延时标准差delay_standard_deviation
        self.declare_parameter("delay_type", 1)  # 1 代表常延时
        self.declare_parameter("average_delay_time", 2.0)
        self.declare_parameter("delay_standard_deviation", 0.2)
        self.declare_parameter("input_delay_topic", "input_delay_topic")
        self.declare_parameter("output_delay_topic", "output_delay_topic")

        # 获取输入话题名称
        input_topic = self.get_parameter('input_delay_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_delay_topic').get_parameter_value().string_value
        # 创建一个订阅者，用于接收消息并产生延时模拟
        self.subscription = self.create_subscription(Float64MultiArray, input_topic, self.listener_callback, 10)
        # 回调函数相当于一个死循环
        self.declare_parameter("receive_frequency", 200.0)  # 0.005s, must be same with T in normal_talker
        # 发布者：用于转发延时消息到公共的话题上
        self.publisher_ = self.create_publisher(Float64MultiArray, output_topic, 10)
        

    # 测试文件配置
    def test(self):
        self.get_logger().info('Hello world!')
    
    # 初始化变量
    def initiate_param(self):
        # oldest_message_pos: 记录最旧的消息位置，用于存储和调用
        # newest_message_pos: 记录最新的消息位置，用于存储和调用
        # actual_oldest_message_num: 实际的最旧的消息序号
        # actual_newest_message_num: 实际的最新的消息序号
        self.total_message_number = 0  # 记录接收消息总数
        self.delay_message_num = -10  # 当前延时下应发送的消息序号
        self.last_published_message_num = -1  # # 上一次采样的延时下发送的消息序号
        self.ii = 0
        self.max_memory = 6001  # 消息存储数量上限
        initial_value = [1.1, 1.1]
        # 创建储存消息的变量
        for jj in range(self.max_memory):
            self.my_name = str(jj)
            self.int_list_param = rclpy.Parameter(name = self.my_name, value = initial_value)
            self.set_parameters([self.int_list_param])

    def cal_delay_time(self):
        # 获取参数
        my_delay_type = self.get_parameter('delay_type').get_parameter_value().integer_value
        my_average_delay_time = self.get_parameter('average_delay_time').get_parameter_value().double_value
        if my_delay_type == 1:
            return my_average_delay_time
        else:
            my_delay_standard_deviation = self.get_parameter('delay_standard_deviation').get_parameter_value().double_value
            return random.gauss(my_average_delay_time, my_delay_standard_deviation)
        
    
    def listener_callback(self, msg):
        # 读取消息
        self.my_message = msg.data
        # 计算最新、旧的消息存储位置
        if self.total_message_number <= self.max_memory - 1:
            oldest_message_pos = 0
            newest_message_pos = self.total_message_number
            # 计算实际的最新、旧消息序号
            actual_oldest_message_num = 0
            actual_newest_message_num = self.total_message_number
        else:
            oldest_message_pos = (self.total_message_number + 1) % self.max_memory
            newest_message_pos = self.total_message_number % self.max_memory
            # 计算实际的最新、旧消息序号
            actual_newest_message_num = self.total_message_number
            actual_oldest_message_num = self.total_message_number - (self.max_memory-1)

        # 在newest_message_pos位置存储新消息，打印结果
        # 修改参数
        temp_name = str(newest_message_pos)
        self.set_parameters([rclpy.Parameter(temp_name, value = self.my_message)])
        # 获取参数
        temp_message = self.get_parameter(temp_name)
        # 发布参数
        self.get_logger().info(f'Delayer recieve: {temp_message.value}')

        # 计算当前delay
        delay = self.cal_delay_time()
        # 消息发布
        # 根据延时确定发布消息的位置
        my_receive_frequency = self.get_parameter('receive_frequency').get_parameter_value().double_value
        self.delay_message_num = actual_newest_message_num - math.ceil(my_receive_frequency * delay)
        # 若发布位置 < 最旧消息位置（对应开头和延时超过几十秒），则不发布消息，上次发布序号不更新
        # 若本次延时消息序号 > 上次延时消息序号，则发布消息；否则不发布，上次发布序号不更新
        if (self.delay_message_num >= actual_oldest_message_num and self.delay_message_num > self.last_published_message_num):
            # 打印上次发布到本次发布间的所有消息
            jj = self.last_published_message_num + 1
            while jj <= self.delay_message_num:
                # 在发布消息时，确保pos位于0~1000内
                if jj < 0:
                    jj = jj + self.max_memory

                # 获取参数
                temp_name = str(jj % self.max_memory)
                temp_message = self.get_parameter(temp_name)
                # 发布参数
                self.get_logger().info(f'Delayer send: {temp_message.value}')  # 发布到终端
                # 转发到公共话题
                delay_msg = Float64MultiArray()
                delay_msg.data = temp_message.value
                self.publisher_.publish(delay_msg)

                jj = jj + 1
            self.last_published_message_num = self.delay_message_num
        else:
            self.get_logger().info('nothing heard')
        # 更新变量
        self.ii = self.ii + 1
        self.total_message_number = self.total_message_number + 1
        # 防止消息序号的数值溢出
        if self.total_message_number > 5 * self.max_memory:
            self.total_message_number = self.total_message_number - self.max_memory
            self.last_published_message_num = self.last_published_message_num - self.max_memory


def main():
    rclpy.init()
    delasten = python_delayed_listener()
    delasten.test()
    delasten.initiate_param()

    rclpy.spin(delasten)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
