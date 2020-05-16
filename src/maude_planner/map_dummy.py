import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import array


class MapDummyPublisher(Node):
    """
    Dummy node for testing purposes that publish a small map (OccupancyGrid) in the topic /map
    """

    def __init__(self):
        super().__init__('map_dummy_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.data = array.array('b')  # Array of unsigned integers of 1 byte
        occupancy_grid.data.extend([0, 0,   0, 0,
                                    0, 100, 0, 0,
                                    0, 0,   0, 0,
                                    0, 0,   0, 0])
        occupancy_grid.info.resolution = 1.0
        occupancy_grid.info.width = 4
        occupancy_grid.info.height = 4
        self.get_logger().info('Sending map to /map: {}'.format(occupancy_grid))
        self.publisher_.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapDummyPublisher()
    rclpy.spin(map_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
