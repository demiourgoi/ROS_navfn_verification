import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseDummyPublisher(Node):
    """
    Dummy node for testing purposes that publish an AMCL pose (PoseWithCovarianceStamped) in the topic /amcl_pose
    """

    def __init__(self):
        super().__init__('pose_dummy_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0  # Represents 0 degree
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending pose to /amcl_pose: {}'.format(msg))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PoseDummyPublisher()
    rclpy.spin(pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
