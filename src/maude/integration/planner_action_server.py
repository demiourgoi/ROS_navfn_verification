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
#    $ ros2 action send_goal ComputePathToPose nav2_msgs/action/ComputePathToPose "{pose: {header: {stamp: {}, frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math, sys

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

# I couldn't import ROS2 tf2_ros2 module :-(
import pyquaternion

import array

import maude

"""
TODO: 
 * Use tf2_ros to transform degress <-> quaternions (using pyquaternion for now)
 * Translate between coordinate frames when needed (tf2)
"""

class MapProvider(maude.Hook):
    """Handle map queries from the special Maude operator open2?"""

    def __init__(self, planner, module):
        super().__init__()

        self.planner = planner

        # Store the true and false terms to be returned by the special
        # operator. Alternatively, term-hook bindings could have been
        # used, but this will be probably slighly faster and it is safe
        # as long as the special operator is only reduced in 'module'.
        self.true_term = module.parseTerm('true')
        self.false_term = module.parseTerm('false')

        self.true_term.reduce()
        self.false_term.reduce()

    def run(self, term, data):
        # The operator arguments are the 2D position and the map side
        # op open2? : Float Float Float -> Bool
        x, y, ncols = [int(arg) for arg in term.arguments()]
        # Map entries are considered opened when the cost is below 50
        # return data.getTerm('trueTerm' if self.planner.occupancy_grid.data[x + y * ncols] < 50
        #                               else 'falseTerm')
        return self.true_term if self.planner.occupancy_grid.data[x + y * ncols] < 254 \
          else self.false_term


class MapProviderGet(maude.Hook):
    """Handle map queries from the special Maude operator get"""

    def __init__(self, planner):
        super().__init__()
        self.planner = planner
        self.cache = dict()  # Dictionary int in [0..255] -> Maude term representing the corresponding Float
        for i in range(0, 256):
            self.cache[i] = self.planner.astar_module.parseTerm(str(float(i)))
    
    def run(self, term, data):
        try:
            _, x, y, ncols = [int(arg) for arg in term.arguments()]
            cell_value = self.planner.occupancy_grid.data[y * ncols + x]
            ret_term = self.cache[cell_value]
            return ret_term
        except Exception as e:
            print('hook:', e)


class MaudePlanner(Node):
    ASTAR_MAUDE_PATH = {
        'a*': '../astar.maude',
        'pot': '../astar_no_turnNavFnPlanner.maude',
        'ros': '../astar_no_turnNavFnPlanner.maude'
    }
    
    ASTAR_OPNAME = {
        'a*': 'a*',
        'pot': 'a*basic',
        'ros': 'a*'
    }

    def __init__(self, implementation='a*', map_in_python=True):
        super().__init__('maude_planner_action_server') # Node name, could be changed to fit the BT

        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH[implementation])
        self.astar_module = maude.getModule('ASTAR')

        if self.astar_module is None:
            self.get_logger().fatal('Cannot find Maude ASTAR module in {}'.format(self.ASTAR_MAUDE_PATH[implementation]))
        else:
            self.get_logger().info('Maude planner node is ready')

        self.occupancy_grid = None  # It will be read and updated from the map topic
        self.maude_map = None
        self.amcl_pose = None       # It will be read and updated from the topic /amcl_pose

        # Configures the action server to respond to ComputePathToPose actions
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'ComputePathToPose',
            self.action_callback)

        # Listen to map topic
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.map_callback,
            10)  # Queue size

        # Listen to topic /amcl_pose
        self._amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10)  # Queue size

        # Plan publisher (so that it is represented in the view)
        self._plan_publisher = self.create_publisher(
            Path,
            '/plan',
            10)

        # Connect the Maude special operator to the map
        self.map_in_python = map_in_python

        if map_in_python:
            if implementation == 'a*':
                self.map_hook = MapProvider(self, self.astar_module)
                maude.connectEqHook('open2?', self.map_hook)
            else:
                self.map_hook = MapProviderGet(self)
                maude.connectEqHook('get', self.map_hook)

        # Find sorts and operators needed to construct the a* term
        # (once for all)
        m = self.astar_module
        pose_kind    = m.findSort('Pose').kind()
        costmap_kind = m.findSort('CostMap').kind()
        float_kind   = m.findSort('Float').kind()
        path_kind    = m.findSort('Path').kind()
        int_kind     = m.findSort('IntList').kind()

        #         Init Goal         NumRow NumCol
        # op a* : Pose Pose CostMap  Float  Float -> Path .
        self.astar_symb   = m.findSymbol(self.ASTAR_OPNAME[implementation], [pose_kind, pose_kind, costmap_kind, float_kind, float_kind], path_kind)
        self.intlist_symb = m.findSymbol('_`,_', [int_kind] * 2, int_kind)
        self.cmap_symb    = m.findSymbol('`{_`}', [int_kind], costmap_kind)

        # Constants that will be used multiple times
        self.zero_term = m.parseTerm('0')
        self.one_term  = m.parseTerm('1')
        self.mtIL_term = m.parseTerm('mtIL')

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
        maude_cells = [('0' if c < 50 else '1') for c in map_data]
        cells_str = ", ".join(maude_cells)
        return "{" + cells_str + "}"
            
    def pose_to_maude(self, p):
        """
        Translates a geometry_msgs/Pose to its Maude representation in a string
        :param p geometry_msgs/Pose
        :return str Maude term
        """
        origin = self.occupancy_grid.info.origin.position
        resolution = self.occupancy_grid.info.resolution

        return "{{{}, {}, {}}} {}".format(round((p.position.x - origin.x) / resolution, 0),
                                          round((p.position.y - origin.y) / resolution, 0),
                                          round((p.position.z - origin.z) / resolution, 0),
                                          self.quaternion_to_angle(p.orientation))

    def angle_to_quaternion(self, angle):
        """
        Method to translate angles in degrees over Y axis to quaternions
        :param angle (int) degrees wrt Y axis
        :return geometry_msgs/Quaternion
        """
        pyq = pyquaternion.Quaternion(axis=[0.0, 0.0, 1.0], degrees=angle) # Rotation in degrees over Z axis

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
        # Angles must be over Z axis or there is no rotation
	    #assert(list(map(abs, pyq.axis)) == [0.0, 0.0, 1.0] or pyq.degrees == 0.0)
	    # Approximate to the coordinate axes or diagonals
        return (45 * round(int(pyq.degrees) / 45)) % 360
        #return int(pyq.degrees)
     
    def maude_to_pose(self, maude_pose):
        """
        :param maude_pose Maude term representing a pose, of the form "{2.0,2.0,0.0} 90"
        :return geometry_msgs/Pose
        """
        p = PoseStamped()

        pose_parts = list(maude_pose.arguments())
        point = pose_parts[0]
        angle = int(pose_parts[1])

        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin.position
        
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.occupancy_grid.header.frame_id

        point_parts = list(point.arguments())
        p.pose.position.x = origin.x + resolution * float(point_parts[0])
        p.pose.position.y = origin.y + resolution * float(point_parts[1])
        p.pose.position.z = origin.z + resolution * float(point_parts[2])
        p.pose.orientation = self.angle_to_quaternion(angle)

        return p
        
    def maude_to_path(self, path_term):
        """
        Method to parse a Maude term representing a Path and generate a geometry_msgs.msg.Path
        :param path_term Maude term containing a path
        :return nav2_msgs/Path
        """
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.occupancy_grid.header.frame_id

        for maude_pose in path_term.arguments():
            # The Maude path ends with noPath
            if str(maude_pose.symbol()) != 'noPath':
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

        # Build the IntList with the costmap data
        # (if the complete map is given to Maude)
        map_list = self.mtIL_term

        if not self.map_in_python:
            for c in self.occupancy_grid.data:
                    if c < 50:
                            map_list = self.intlist_symb(map_list, self.zero_term)
                    else:
                            map_list = self.intlist_symb(map_list, self.one_term)

        # Build the a* term with makeTerm
        term = self.astar_symb(
                self.astar_module.parseTerm(maude_pose_ini),
                self.astar_module.parseTerm(maude_pose_fin),
                self.cmap_symb(map_list),
                self.astar_module.parseTerm(str(float(self.occupancy_grid.info.height))),
                self.astar_module.parseTerm(str(float(self.occupancy_grid.info.width)))
        )
        term.reduce()
        self.get_logger().info('Maude path found: {}'.format(term))

        plan = self.maude_to_path(term)

        # Publish the plan to be drawn by RViz
        self._plan_publisher.publish(plan)

        return plan

    def map_callback(self, map):
        self.occupancy_grid = map  # Also stores the original ROS map to keep all the details (for future extensions)
        #self.maude_map = self.map_data_to_maude(map.data)
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

        if self.amcl_pose and self.occupancy_grid:
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
    import argparse
    
    parser = argparse.ArgumentParser(description='Maude planner action server')
    parser.add_argument(
        '-i', '--implementation',
        help='Planning algorithm implementation',
	choices=['a*', 'pot', 'ros'],
	default='a*',
    )
    pargs = parser.parse_args()

    rclpy.init(args=args)

    maude_planner = MaudePlanner(pargs.implementation)

    rclpy.spin(maude_planner)


if __name__ == '__main__':
    main()
