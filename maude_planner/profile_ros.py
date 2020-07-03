# -*- coding: utf-8 -*-

import profile_maude

import rclpy, random, pyquaternion, array, time, math, csv
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action  import ComputePathToPose
from nav_msgs.msg      import OccupancyGrid

default_costmap = profile_maude.default_costmap
TEST_SUITE = profile_maude.TEST_SUITE

class ProfilerNode(Node):
    '''Profiler of the Maude planner'''

    def __init__(self):
        super().__init__('maude_profiler_node')
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.map_publisher  = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 10)
        self.action_client  = ActionClient(self, ComputePathToPose, '/ComputePathToPose')

        self.timer = self.create_timer(0.1, self.profile)

        # Index in the test suite
        self.index = 0
        # CSV where result are saved
        self.csvfile = open('results.csv', 'w')
        self.csvwriter = csv.writer(self.csvfile)

    def convert_position(self, x, y):
        '''Convert from map indices to the coordinates used by ROS'''

        return default_costmap.x0 + default_costmap.resolution * x, \
               default_costmap.y0 + default_costmap.resolution * y

    @staticmethod
    def random_position():
        '''Generate a random position inside the navigable region of the default map'''

        return float(random.randrange(158, 158 + 85)), \
               float(random.randrange(138, 138 + 91)), \
               random.uniform(0.0, 360.0)

    @staticmethod
    def angle_to_quaternion(angle):
        '''Convert angles in degrees to quaternions'''

        pyq = pyquaternion.Quaternion(axis=[0.0, 0.0, 1.0], degrees=angle)

        q = Quaternion()
        q.x, q.y, q.z, q.w = pyq.x, pyq.y, pyq.z, pyq.w
        return q

    @staticmethod
    def quaternion_to_angle(q):
        '''Convert angles in degrees to quaternions'''

        return pyquaternion.Quaternion(q.w, q.x, q.y, q.z).degrees

    def send_initial_pose(self, position):
        '''Send the initial pose to planner'''

        x, y, theta = position
        x, y = self.convert_position(x, y)

        msg = PoseWithCovarianceStamped()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.angle_to_quaternion(theta)

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        self.pose_publisher.publish(msg)

    def send_map(self):
        '''Send the map to the planner'''
        ogrid = OccupancyGrid()

        ogrid.info.origin.position.x = default_costmap.x0
        ogrid.info.origin.position.y = default_costmap.y0
        ogrid.info.origin.position.z = 0.0
        ogrid.data = array.array('b')  # Array of unsigned integers of 1 byte
        ogrid.data.extend(default_costmap.data)
        ogrid.info.resolution = default_costmap.resolution
        ogrid.info.width = default_costmap.width
        ogrid.info.height = default_costmap.height

        ogrid.header.frame_id = 'map'
        ogrid.header.stamp = self.get_clock().now().to_msg()

        self.map_publisher.publish(ogrid)

    def compute_path(self, position):
        '''Ask the planner to calculate the path'''
        x, y, theta = position
        x, y = self.convert_position(x, y)

        goal = ComputePathToPose.Goal()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.angle_to_quaternion(theta)
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.action_client.wait_for_server()
        time.sleep(1.0) 
        print('Servidor esperado')

        self.start_time = time.perf_counter()
        self.future = self.action_client.send_goal_async(goal, feedback_callback=self.computed_path_callback)

        self.future.add_done_callback(self.accepted_action_callback)

    def accepted_action_callback(self, data):
        '''Callback called when the ComputePathToPose message is accepted'''

        result = data.result()
        if result.accepted:
            result.get_result_async().add_done_callback(self.computed_path_callback)
        else:
            print('Petición rechazada')

    def computed_path_callback(self, data):
        '''Callback called when the ComputePathToPose message is solved'''

        end_time = time.perf_counter()
        result = data.result().result.path

        hmtime = end_time - self.start_time
        distance = self.calculate_length(result.poses)
        rotation, numrot = self.calculate_rotation(result.poses)

        print('Time:', hmtime)
        print('Length:', distance)
        print('Distance:', self.distance2)
        print('Distance 1:', self.distance1)
        print('Acc. rotation:', rotation)
        print('# rotations:', numrot)

        self.csvwriter.writerow([True] + list(TEST_SUITE[self.index][0]) + list(TEST_SUITE[self.index][1]) + [hmtime, distance, self.distance2, self.distance1, rotation, numrot])

        # Starts again
        self.index = self.index + 1
        self.timer = self.create_timer(0.2, self.profile)

    def calculate_length(self, poses):
        '''Calculate the length of a path'''

        if len(poses) == 0:
        	return 0.0

        x = poses[0].position.x
        y = poses[0].position.y
        s = 0.0

        for pose in poses:
            s += math.sqrt((pose.position.x - x) ** 2 + (pose.position.y - y) ** 2)
            x = pose.position.x
            y = pose.position.y

        return s

    def calculate_rotation(self, poses):
        '''Calculate the length of a path'''

        if len(poses) == 0:
        	return 0.0, 0

        t = self.quaternion_to_angle(poses[0].orientation)
        s = 0.0
        k = 0

        for pose in poses:
            nt = self.quaternion_to_angle(pose.orientation)
            s += abs(nt - t)
            if nt != t:
                k = k + 1
            t = nt

        return s, k

    def calculate_distance(self, point1, point2, norm=2):
    	'''Calculate the distance between two points'''

    	return (abs(point2[0] - point1[0]) ** norm + abs(point2[1] - point1[1]) ** norm) ** (1.0 / norm)

    def profile(self):
        '''Profile the Maude planner'''

        self.destroy_timer(self.timer)
        self.timer = None

        if self.index < len(TEST_SUITE):
            initial = TEST_SUITE[self.index][0]
            end     = TEST_SUITE[self.index][1]

            print(initial, 'to', end)

            self.send_initial_pose(initial)
            self.send_map()

            self.distance2 = self.calculate_distance(initial, end)
            self.distance1 = self.calculate_distance(initial, end, norm=1)

            self.compute_path(end)
        else:
            self.csvfile.close()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    profiler = ProfilerNode()
    rclpy.spin(profiler)

if __name__ == '__main__':
    main()
