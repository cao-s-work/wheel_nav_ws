#!/usr/bin/env python3
"""简单键盘遥控 — 输入命令 + Enter，q 退出。
  w/up: 前进   s/down: 后退   a/left: 左转   d/right: 右转
  space: 停车   f: 急停(趴下)   +/-: 调速
"""
import sys
import time
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__("simple_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speed = 0.15
        self.turn = 0.30

    def send(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def stop(self):
        self.send(0.0, 0.0)


def main():
    rclpy.init()
    node = SimpleTeleop()
    print("w/s 进退 | a/d 转向 | space 停 | f 趴下 | +/- 调速 | q 退出")
    print(f"当前速度: 线={node.speed:.2f}  角={node.turn:.2f}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd in ("q", "quit"):
                    break
                elif cmd in ("w", "up"):
                    node.send(node.speed, 0.0)
                elif cmd in ("s", "down"):
                    node.send(-node.speed, 0.0)
                elif cmd in ("a", "left"):
                    node.send(0.0, node.turn)
                elif cmd in ("d", "right"):
                    node.send(0.0, -node.turn)
                elif cmd in (" ", "space", ""):
                    node.stop()
                elif cmd in ("f", "estop"):
                    node.stop()
                    print("急停 — 请手动调 service 趴下")
                elif cmd == "+":
                    node.speed = min(0.60, node.speed + 0.05)
                    node.turn = min(0.80, node.turn + 0.05)
                    print(f"速度: 线={node.speed:.2f}  角={node.turn:.2f}")
                elif cmd == "-":
                    node.speed = max(0.05, node.speed - 0.05)
                    node.turn = max(0.10, node.turn - 0.05)
                    print(f"速度: 线={node.speed:.2f}  角={node.turn:.2f}")
                else:
                    print(f"未知命令: {cmd}")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
