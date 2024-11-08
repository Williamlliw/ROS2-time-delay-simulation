from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类
def generate_launch_description():             # 自动生成launch文件的函数
    # 定义发送端话题名称
    input_topic = "my_delay_topic"
    # 定义接收端话题名称
    output_topic = "out_delay_topic"
    return LaunchDescription([                 # 返回launch文件的描述信息
	Node(                                  # 配置一个节点的启动
		package='delay_simulation',          # 节点所在的功能包
		executable='talker', # 节点的可执行文件 
		parameters=[
		    {'input_delay_topic': input_topic},
            {'output_delay_topic': output_topic}
		]
	),        
	Node(                                  # 配置一个节点的启动
		package='delay_simulation',          # 节点所在的功能包
		executable='my_delayer', # simulate variable time delay
		parameters=[
            {'delay_type': 0},  # 延时类型delay_type, 1 代表常延时，其它整数代表变延时
            {'average_delay_time': 0.03},  # 平均延时average_delay_time
            {'delay_standard_deviation': 0.005},  # 延时标准差delay_standard_deviation
            {'receive_frequency': 200.0},  # 消息采样频率，必须与发送频率相同
            {'input_delay_topic': input_topic},
            {'output_delay_topic': output_topic}
            ]
	),
    ])

