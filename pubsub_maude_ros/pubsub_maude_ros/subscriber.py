import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import maude


class MaudeListener(Node):

    def __init__(self):
        super().__init__('maude_listener')
        self.subscription = self.create_subscription(
            String,
            'maude_msgs',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.maude_nat = maude.getModule('NAT')

    def listener_callback(self, msg):
        t = self.maude_nat.parseTerm('{} * {}'.format(msg.data, msg.data))
        t.reduce()
        self.get_logger().info('Recibido "{}" y elevado al cuadrado "{}"'.format(msg.data, t))


def main(args=None):
    maude.init()
    
    rclpy.init(args=args)

    maude_subscriber = MaudeListener()

    rclpy.spin(maude_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    maude_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
