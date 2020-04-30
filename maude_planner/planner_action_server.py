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
from nav2_msgs.msg import Path
from geometry_msgs.msg import Pose

import maude

"""
TODO: 
 * conectar con el Maude loco de Adrián usando la librería de Rubén
 * ser capaz de pasar de orientación en grados a cuaterniones usando algo de ROS
"""


class MaudePlanner(Node):
    ASTAR_MAUDE_PATH = '../maude/astar.maude'

    def __init__(self):
        super().__init__('maude_planner_action_server') # Node name, could be changed to fit the BT
        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH)
        self.astar_module = maude.getModule('ASTAR')
        self.costmap = None  # Stores the latest version of the costmap as a 
                             # string, using Maude representation
        self.costmap_numrow = 0.0  # Yes, they need to be float numbers ;-)
        self.costmap_numcol = 0.0                   
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'ComputePathToPose',
            self.execute_callback)
            
    def get_current_pose(self):
        # TODO: constant function, complete!
        p = Pose()
        p.position.x = 1.0
        p.position.y = 2.0
        p.position.z = 3.0
        p.orientation = self.angle_to_quaternion(angle)
        return p
        
    def costmap_to_maude(self, costmap):
        """
        :param costmap nav2_msg.CostMap <-- revisar
        :return (str, float, float)
        """
        # TODO: take a real costmap and translate
        return ("{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}", 4.0, 4.0)
            
    def pose_to_maude(self, pose):
        # TODO: constant function, complete!
        return "{0.0, 0.0, 0.0} 90"
        
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
        return p
     
    def maude_to_pose(self, angle, pose_str):
        """
        pose_str is a string of the form "({2.0,2.0,0.0} 90)"
        """
        # TODO: constant function, complete!
        p = Pose()
        p.position.x = 1.0
        p.position.y = 2.0
        p.position.z = 3.0
        p.orientation = self.angle_to_quaternion(angle)
        return p  
        
    def maude_to_path(self, path_str):
        """
        Method to parse a string with a path returned by Maude and generate a 
        Path message
        """
        # TODO: constant function, complete!
        p = Path()
        return p

    def execute_callback(self, goal_handle):
        final_pose = goal_handle.request.pose.pose
        self.get_logger().info('Computing path to {}'.format(final_pose))
        
        # Generate Response
        path = self.maude_to_path("")  # We need to call maude
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
