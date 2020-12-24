import rclpy

from sensor_msgs.msg import Image

def callback(msg):
    node.get_logger().info('Received {}'.format(msg.header))

def main():
    rclpy.init()

    global node
    node = rclpy.create_node('tester')

    subscription = node.create_subscription(
        Image, '/camera/image_raw', callback, 10)
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
