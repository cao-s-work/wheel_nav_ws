# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import struct
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
# class SerialTwistPublisher(Node):
#     def __init__(self):
#         super().__init__('serial_twist_publisher')

#         # 参数配置
#         self.declare_parameter('port', '/dev/ttyACM0')
#         self.declare_parameter('baudrate', 115200)
#         self.declare_parameter('linear_scale', 1000.0)  # m/s -> mm/s
#         self.declare_parameter('angular_scale', 1000.0)  # rad/s -> milli rad/s or deg/s ?

#         port = self.get_parameter('port').get_parameter_value().string_value
#         baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
#         self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
#         self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
#          # 添加一个定时器，每秒打印一次日志，确认 spin() 正常工作
#         self.timer = self.create_timer(1.0, self.timer_callback)

#         # 初始化串口
#         try:
#             self.ser = serial.Serial(port, baudrate, timeout=1)
#             self.get_logger().info(f'串口 {port} 已打开，波特率 {baudrate}')
#         except Exception as e:
#             self.get_logger().error(f'无法打开串口: {e}')
#             raise
#         # 使用 best effort 的 QoS 设置
#         # 使用与发布者一致的 QoS 设置
#         qos = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
#         )

#         self.subscription = self.create_subscription(
#             Twist,
#             'cmd_vel',
#             self.twist_callback,
#             qos
#         )
#         # # 订阅 /cmd_vel
#         # self.subscription = self.create_subscription(
#         #     Twist,
#         #     'cmd_vel',
#         #     self.twist_callback,
#         #     10
#         # )

#     def timer_callback(self):
#         self.get_logger().info("【DEBUG】spin() 正常运行中...")

#     def twist_callback(self, msg):
#         vx = msg.linear.x
#         wz = msg.angular.z

#         # 转换为整数并缩放
#         speed = int(vx * self.linear_scale)
#         angular = int(wz * self.angular_scale)

#         # 限制范围（可选）
#         speed = max(min(speed, 0x7FFF), -0x8000)
#         angular = max(min(angular, 0x7FFF), -0x8000)

#         # 转换为 2 字节（有符号，大端）
#         speed_bytes = struct.pack('>h', speed)
#         angular_bytes = struct.pack('>h', angular)

#         # 构造数据包
#         packet = bytes([
#             0xCC,
#             speed_bytes[0], speed_bytes[1],
#             angular_bytes[0], angular_bytes[1],
#             0xEE
#         ])
        

#     # 模拟发送数据
#         # self.get_logger().info(f'模拟发送数据: {packet.hex()}')
#         try:
#             self.ser.write(packet)
#             self.get_logger().debug(f'发送数据: {packet.hex()}')
#         except Exception as e:
#             self.get_logger().error(f'串口写入失败: {e}')

# def main():
#     rclpy.init()
#     node = SerialTwistPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# # ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
# # linear:
# #   x: 0.3
# #   y: 0.0
# #   z: 0.0
# # angular:
# #   x: 0.0
# #   y: 0.0
# #   z: 0.5"



#!/usr/bin/env python3

import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist



class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # 参数配置
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('linear_scale', 1000.0)  
        self.declare_parameter('angular_scale', 1000.0)  

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        
                # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'串口 {port} 已打开，波特率 {baudrate}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            raise

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,           # 消息类型
            '/cmd_vel',      # 话题名称
            self.listener_callback,  # 回调函数
            10               # QoS profile depth
        )
        self.get_logger().info("正在监听 /cmd_vel 话题...")

    def listener_callback(self, msg):
        # # 提取线速度和角速度
        # linear_x = msg.linear.x
        # angular_z = msg.angular.z

        # 提取线速度和角速度
        vx = msg.linear.x
        wz = msg.angular.z

        # 缩放并转换为整数（int16）
        speed = int(vx * self.linear_scale)
        angular = int(wz * self.angular_scale)

        # 限制在 int16 范围内 (-32768 ~ 32767)
        speed = max(min(speed, 0x7FFF), -0x8000)
        angular = max(min(angular, 0x7FFF), -0x8000)

        # 转换为 2 字节（有符号，大端）
        speed_bytes = struct.pack('>h', speed)  # > 表示大端，h 表示 int16
        angular_bytes = struct.pack('>h', angular)

        # 构造数据包
        packet = bytes([
            0xCC,
            speed_bytes[0], speed_bytes[1],
            angular_bytes[0], angular_bytes[1],
            0xEE
        ])

        # 发送数据
        self.ser.write(packet)
        self.get_logger().debug(f'发送数据包: {packet.hex()}')
        # try:
        #     self.ser.write(packet)
        #     self.get_logger().debug(f'发送数据包: {packet.hex()}')
        # except Exception as e:
        #     self.get_logger().error(f'串口写入失败: {e}')


        # 打印出来
        self.get_logger().info(f'接收到速度指令: 线速度 x={speed:.2f}, 角速度 z={angular:.2f}')


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()

    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        pass


    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()