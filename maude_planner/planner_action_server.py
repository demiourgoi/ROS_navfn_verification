# -*- coding: utf-8 -*-

"""
2020 - Enrique Martín <emartinm@ucm.es>

ROS2 planner as an action server using a Maude implementation of A*.

Action definition in https://github.com/ros-planning/navigation2/blob/master/nav2_msgs/action/ComputePathToPose.action
"""

# Test from the terminal:
# 1) $ source /opt/ros/dashing/setup.bash 
# 2) $ python3 planner_action_server.py
# 3) $ ros2 action send_goal ComputePathToPose nav2_msgs/action/ComputePathToPose "{pose: {header: {stamp: {}, frame_id: "vete"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}}"

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Path, Costmap
from geometry_msgs.msg import Pose, Quaternion

import array

import re

import maude

"""
TODO: 
 * ser capaz de pasar de orientación en grados a cuaterniones usando algo de ROS
 * pasar las poses originales a posiciones del mapa, y traducir poses del Path 
   al marco original
"""


class MaudePlanner(Node):
    ASTAR_MAUDE_PATH = '../maude/astar.maude'

    def __init__(self):
        super().__init__('maude_planner_action_server') # Node name, could be changed to fit the BT

        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH)        
        self.astar_module = maude.getModule('ASTAR')
        
        self.pose_pattern = re.compile(r'\(\{([0-9]+\.[0-9]+),([0-9]+\.[0-9]+),([0-9]+\.[0-9]+)\} ([0-9]+)\)')
        self.maude_path_pattern = re.compile(r'{\d+\.\d+,\d+\.\d+,\d+\.\d+} \d+')
    
        self.costmap, self.costmap_numrow, self.costmap_numcol = self.get_current_map()
        # Stores the latest version of the costmap as a string, using Maude representation
        # as well as the number of rows and columns
        # TODO: Should it be updated if the map changes? Can it change?
        
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'ComputePathToPose',
            self.execute_callback)
            
    def costmap_to_maude(self, costmap):
        """
        :param costmap nav2_msg.CostMap
        :return (str, float, float): string representing the map (0 empty, 1 obstacle),
                number of rows and number of columns
        """
        num_row = costmap.metadata.size_y
        num_col = costmap.metadata.size_x
        
        # TODO: They should be used some day
        resolution = costmap.metadata.resolution
        origin  = costmap.metadata.origin
        

        # https://github.com/ros-planning/navigation2/blob/aaa97902c1e1bb9a509c4ee0d88b880afb528f20/nav2_util/src/costmap.cpp#L27
        # * free positions (0) are marked as free to Maude (0)
        # * other positions (no_information, lethal_obstacle, inscribed_inflated_obstacle and medium_cos)
        #   are marked as obstacles to Maude (1)
        maude_cells = [('0' if c == 0 else '1') for c in costmap.data]
        cells_str = ", ".join(maude_cells)
        
        return ("{" + cells_str + "}", float(num_row), float(num_col))
        
    def get_current_map(self):
        # TODO: take the map from a real place instead of creating a fake Costmap!        
        # It seems map_server provides maps using a topic and also a service:
        # https://index.ros.org/p/nav2_map_server/github-ros-planning-navigation2/#dashing
        
        # Fake map that should be obtained from the map server
        fake = Costmap()
        fake.metadata.size_y = 4
        fake.metadata.size_x = 4
        fake.data = array.array('B')  # Array of unsigned integers of 1 byte
        fake.data.extend([0, 0,   0, 0, 
                          0, 254, 0, 0,
	                      0, 0,   0, 0,
 	                      0, 0,   0, 0])
	                   
        return self.costmap_to_maude(fake)
            
    def get_current_pose(self):
        # TODO: constant function, complete!
        # It needs to listen to some topic? Call some action?
        p = Pose()
        p.position.x = 2.0
        p.position.y = 2.0
        p.position.z = 0.0
        p.orientation = self.angle_to_quaternion(0)
        return p
            
    def pose_to_maude(self, p):
        return "{{{}, {}, {}}} {}".format(p.position.x, p.position.y, p.position.z, 
                                           self.quaternion_to_angle(p.orientation))
        
    def angle_to_quaternion(self, angle):
        """
        Method to translate angles in grades to quaternions
        """
        # TODO: constant function, complete!
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 0.0
        return q
        
    def quaternion_to_angle(self, q):
        return 90
     
    def maude_to_pose(self, pose_str):
        """
        pose_str is a string of the form "({2.0,2.0,0.0} 90)"
        """
        m = re.match(self.pose_pattern, pose_str.strip())
        p = Pose()
        p.position.x = float(m.group(1))
        p.position.y = float(m.group(2))
        p.position.z = float(m.group(3))
        p.orientation = self.angle_to_quaternion(float(m.group(4)))
        return p  
        
    def maude_to_path(self, path):
        """
        Method to parse a string with a path returned by Maude and generate a 
        Path message
        :param path Maude term containing a path. I will contain at least one pose
        """
        # TODO: create path from pose strings
        poses = re.findall(self.maude_path_pattern, str(path))
        print(poses)
        p = Path()
        return p
    
    def compute_path_in_maude(self, pose_ini, pose_fin):
        """
        Calls A* to compute a path from pose_ini to pose_fin
        """
        maude_pose_ini = self.pose_to_maude(pose_ini)
        maude_pose_fin = self.pose_to_maude(pose_fin)
        self.get_logger().info('Computing Path from {} to {}'.format(maude_pose_ini, maude_pose_fin))
        
        term = self.astar_module.parseTerm('a*({}, {}, {}, {}, {})'.format(
          maude_pose_ini, maude_pose_fin, self.costmap, self.costmap_numrow, 
          self.costmap_numcol))
        term.reduce()
        self.get_logger().info('Maude path found: {}'.format(term))
        
        return self.maude_to_path(term)

    def execute_callback(self, goal_handle):
        ini_pose = self.get_current_pose()
        final_pose = goal_handle.request.pose.pose
        self.get_logger().info('Computing path from {} to {}'.format(ini_pose, final_pose))
        
        # Generate Response
        path = self.compute_path_in_maude(ini_pose, final_pose)
        result = ComputePathToPose.Result()
        result.path = path
        
        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)

    maude_planner = MaudePlanner()

    rclpy.spin(maude_planner)


if __name__ == '__main__':
    main()
