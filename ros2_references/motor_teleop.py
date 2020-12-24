#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def send_message(string):
    msg = String()
    msg.data = string
    
    publisher.publish(msg)
    node.get_logger().info('Publicando: "{}"'.format(string))


def main():
    rclpy.init()
    global node
    global publisher
    node = Node('motor_teleop')
    publisher = node.create_publisher(String, 'motor_command', 10)

    valid_inputs = ('w', 'a', 's', 'd', 'x')

    while rclpy.ok():
        key = input()
        if key in valid_inputs:
            send_message(key)

    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
