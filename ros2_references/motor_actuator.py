# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from std_msgs.msg import String


def direction_callback(msg):
    if msg.data == 'w':
        node.get_logger().info('Robô vai para FRENTE')
    elif msg.data == 'a':
        node.get_logger().info('Robô vai pra ESQUERDA')
    elif msg.data == 's':
        node.get_logger().info('Robô vai para TRÁS')
    elif msg.data == 'd':
        node.get_logger().info('Robô vai para DIREITA')
    elif msg.data == 'x':
        node.get_logger().info('Robô PARA')
    else:
        node.get_logger().info('Comando inválido')


def main(args=None):
    rclpy.init(args=args)

    global node
    node = rclpy.create_node('motor_actuator')

    subscription = node.create_subscription(
        String, 'motor_command', direction_callback, 10)
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
