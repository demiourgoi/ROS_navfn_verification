import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from random import randint

import maude


class MaudePublisher(Node):

    def __init__(self):
        super().__init__('maude_publisher')
        self.publisher_ = self.create_publisher(String, 'maude_msgs', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.maude_nat = maude.getModule('NAT')

    def timer_callback(self):
        # Crear un término de diferencia simétrica y lo reduce
        t = self.maude_nat.parseTerm('sd({}, {})'.format(randint(1,100), randint(1,100)))
        t0 = t.copy()
        t.reduce()
        msg = String()
        msg.data = "{} = {}".format(t0, t)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "{}"'.format(msg.data))
        self.i += 1


def main(args=None):
    maude.init()
    
    rclpy.init(args=args)

    maude_publisher = MaudePublisher()

    rclpy.spin(maude_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    maude_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
