# -*- coding: utf-8 -*-

"""
Enrique Martín <emartinm@ucm.es> 2020

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

"""
TODO: 
 * conectar con el Maude loco de Adrián usando la librería de Rubén
 * ser capaz de pasar de orientación en grados a cuaterniones usando algo de ROS
"""


class MaudePlanner(Node):

    def __init__(self):
        super().__init__('maude_planner_action_server') # Node name, could be changed to fit the BT
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'ComputePathToPose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        final_pose = goal_handle.request.pose.pose
        self.get_logger().info('Computing path to {}'.format(final_pose))
        
        # Generate Response
        positions = Path()
        positions.poses = [final_pose, final_pose] # Dummy path
        result = ComputePathToPose.Result()
        result.path = positions
        
        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)

    maude_planner = MaudePlanner()

    rclpy.spin(maude_planner)


if __name__ == '__main__':
    main()
