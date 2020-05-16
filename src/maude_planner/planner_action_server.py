# -*- coding: utf-8 -*-

"""
2020 - Enrique Martín <emartinm@ucm.es>

ROS2 planner as an action server using a Maude implementation of A*.

Action definition in https://github.com/ros-planning/navigation2/blob/master/nav2_msgs/action/ComputePathToPose.action
"""

# Test from the terminal:
# Load ROS environment in all the terminals ($ source /opt/ros/dashing/setup.bash)
# 1) Launch dummy /map publisher:
#    $ python map_dummy.py
# 2) Launch dummy /amcl_pose publisher:
#    $ python3 amcl_pose_dummy.py
# 3) Launch planner action server
#    $ python3 planner_action_server.py
# 4) Wait 5 seconds so that a map is published and received
# 5) Ask the action server:
#    $ ros2 action send_goal ComputePathToPose nav2_msgs/action/ComputePathToPose "{pose: {header: {stamp: {}, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Path, Costmap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped

# I couldn't import ROS2 tf2_ros2 module :-(
import pyquaternion

import array

import maude

"""
TODO: 
 * Use tf2_ros to transform degress <-> quaternions (using pyquaternion for now)
 * Translate between coordinate frames when needed (tf2)
"""


class MaudePlanner(Node):
    ASTAR_MAUDE_PATH = '../maude/astar.maude'

    def __init__(self):
        super().__init__('maude_planner_action_server') # Node name, could be changed to fit the BT

        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH)        
        self.astar_module = maude.getModule('ASTAR')

        self.occupancy_grid = None  # It will be read and updated from the topic /map
        self.maude_map = None
        self.amcl_pose = None       # It will be read and updated from the topic /amcl_pose

        # Configures the action server to respond to ComputePathToPose actions
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'ComputePathToPose',
            self.action_callback)

        # Listen to topic /map
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)  # Queue size

        # Listen to topic /amcl_pose
        self._amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10)  # Queue size
            
    def map_data_to_maude(self, map_data):
        """
        Translates between an array of signed bytes representing a map data to the Maude term
        :param map_data int8[]
        :return str representing the map data (0 empty, 1 obstacle),
        """
        # https://github.com/ros-planning/navigation2/blob/aaa97902c1e1bb9a509c4ee0d88b880afb528f20/nav2_util/src/costmap.cpp#L27
        # * free positions (0) are marked as free to Maude (0)
        # * other positions (no_information, lethal_obstacle, inscribed_inflated_obstacle and medium_cos)
        #   are marked as obstacles to Maude (1)
        maude_cells = [('0' if c == 0 else '1') for c in map_data]
        cells_str = ", ".join(maude_cells)
        return "{" + cells_str + "}"
            
    def pose_to_maude(self, p):
        """
        Translates a geometry_msgs/Pose to its Maude representation in a string
        :param p geometry_msgs/Pose
        :return str Maude term
        """
        return "{{{}, {}, {}}} {}".format(p.position.x, p.position.y, p.position.z, 
                                           self.quaternion_to_angle(p.orientation))
        
    def angle_to_quaternion(self, angle):
        """
        Method to translate angles in degrees over Y axis to quaternions
        :param angle (int) degrees wrt Y axis
        :return geometry_msgs/Quaternion
        """
        pyq = pyquaternion.Quaternion(axis=[0.0, 1.0, 0.0], degrees=angle) # Rotation in degrees over Y axis

        q = Quaternion()
        q.x = pyq.x
        q.y = pyq.y
        q.z = pyq.z
        q.w = pyq.w
        return q
        
    def quaternion_to_angle(self, q):
        """
        :param q geometry_msgs/Quaternion representing an angle (degrees) wrt Y axis
        :return (int) number of degrees wrt Y axis
        """
        pyq = pyquaternion.Quaternion(q.w, q.x, q.y, q.z)
        # Angles must be over Y axis or there is no rotation
        assert(list(pyq.axis) == [0.0, 1.0, 0.0] or pyq.degrees == 0.0)
        return int(pyq.degrees)
     
    def maude_to_pose(self, maude_pose):
        """
        :param maude_pose Maude term representing a pose, of the form "{2.0,2.0,0.0} 90"
        :return geometry_msgs/Pose
        """
        p = Pose()

        pose_parts = list(maude_pose.arguments())
        point = pose_parts[0]
        angle = int(str(pose_parts[1]))

        point_parts = list(point.arguments())
        p.position.x = float(str(point_parts[0]))
        p.position.y = float(str(point_parts[1]))
        p.position.z = float(str(point_parts[2]))
        p.orientation = self.angle_to_quaternion(angle)

        return p  
        
    def maude_to_path(self, path_term):
        """
        Method to parse a Maude term representing a Path and generate a geometry_msgs.msg.Path
        :param path_term Maude term containing a path
        :return nav2_msgs/Path
        """
        p = Path()
        for maude_pose in path_term.arguments():
            p.poses.append(self.maude_to_pose(maude_pose))
        return p
    
    def compute_path_in_maude(self, pose_ini, pose_fin):
        """
        Calls Maude implementation of A* to compute a path from pose_ini to pose_fin,
        returning a geometry_msgs.msg.Path
        :param pose_ini (geometry_msgs.msg.Pose)
        :param pose_fin (geometry_msgs.msg.Pose)
        :returns (geometry_msgs.msg.Path)
        """
        maude_pose_ini = self.pose_to_maude(pose_ini)
        maude_pose_fin = self.pose_to_maude(pose_fin)
        self.get_logger().info('Computing Path from {} to {}'.format(maude_pose_ini, maude_pose_fin))

        term = self.astar_module.parseTerm('a*({}, {}, {}, {}, {})'.format(
            maude_pose_ini,
            maude_pose_fin,
            self.maude_map,
            float(self.occupancy_grid.info.height), # They must be float in the Maude term
            float(self.occupancy_grid.info.width))
        )
        term.reduce()
        self.get_logger().info('Maude path found: {}'.format(term))
        
        return self.maude_to_path(term)

    def map_callback(self, map):
        self.occupancy_grid = map  # Also stores the original ROS map to keep all the details (for future extensions)
        self.maude_map = self.map_data_to_maude(map.data)
        # The map is translated to Maude because it will rarely change and it is an expensive operation
        # that could delay a ComputePathToPose call if lazily delayed
        # self.get_logger().info('Storing map from /map: {}'.format(self.maude_map))

    def amcl_pose_callback(self, pose):
        # self.get_logger().info('Storing current position from /amcl_pose: {}'.format(pose))
        self.amcl_pose = pose
        # This pose is lazily translated to Maude only when required

    def action_callback(self, goal_handle):
        final_pose = goal_handle.request.pose.pose
        self.get_logger().info('Computing path from \n\t{} \n\t\tto\n\t {}'.format(self.amcl_pose, final_pose))

        if self.amcl_pose and self.occupancy_grid and self.maude_map:
            # Generate Response
            path = self.compute_path_in_maude(self.amcl_pose.pose.pose, final_pose)
            result = ComputePathToPose.Result()
            result.path = path
            goal_handle.succeed()
            return result
        else:
            self.get_logger().error('No current pose or map information')
            goal_handle.abort()


def main(args=None):
    rclpy.init(args=args)

    maude_planner = MaudePlanner()

    rclpy.spin(maude_planner)


if __name__ == '__main__':
    main()
