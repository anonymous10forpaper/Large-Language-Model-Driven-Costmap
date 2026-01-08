#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EventSender(Node):
    def __init__(self):
        super().__init__('event_sender')
        self.declare_parameter('event_topic', '/event')
        topic = self.get_parameter('event_topic').value
        self.pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"Publishing events to: {topic}")

    def run(self):
        while rclpy.ok():
            try:
                line = input("Event: ").strip()
                if not line:
                    continue
                if line.lower() in ("q", "quit", "exit"):
                    break
                msg = String()
                msg.data = line
                self.pub.publish(msg)
                print(f"Sent event: {line}")
            except (EOFError, KeyboardInterrupt):
                break

def main():
    rclpy.init()
    node = EventSender()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
