import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class EquipoMinimo(Node):

    def __init__(self):
        super().__init__('equipo_minimo')
        self.subscription = self.create_subscription(
            String,
            'arenga',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('El mister nos ha dicho: "{}"'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = EquipoMinimo()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
