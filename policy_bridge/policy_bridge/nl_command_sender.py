#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NlCommandSender(Node):
    def __init__(self):
        super().__init__('nl_command_sender')
        self.declare_parameter('nl_command_topic', '/nl_command')
        topic = self.get_parameter('nl_command_topic').value
        self.pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"Publishing NL commands to: {topic}")

    def run(self):
        while rclpy.ok():
            try:
                line = input("Command: ").strip()
                if not line:
                    continue
                if line.lower() in ("q", "quit", "exit"):
                    break
                msg = String()
                msg.data = line
                self.pub.publish(msg)
                print(f"sent: {line}")
            except (EOFError, KeyboardInterrupt):
                break

def main():
    rclpy.init()
    node = NlCommandSender()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
