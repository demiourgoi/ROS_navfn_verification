# Maude integration and verification for [ROS Navigation 2](https://navigation.ros.org/)

The Maude implementations of the A* algorithm are in the `src/maude` directory. There are various versions:

* NavFn<sub>A*</sub> is a textbook implementation of the A* algorithm. It can be executed with the `a*` operator of the `ASTAR` module in `astar.maude`.
* NavFn<sub>pot</sub> calculates a potential function imitating the [`NavFn` planner](https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner) of Navigation 2, and then computes the path from the origin to the goal by always taking the neighbor with the lowest potential as the next position. This is the `computePath` function in the `BASIC-TRAVERSE` module of `astar_no_turnNavFnPlanner.maude`.
* NavFn<sub>ROS</sub> imitates the complete `NavFN` implementation, also in the second phase where the path is computed. It is available in the `a*` function of the `ASTAR` module of `astar_no_turnNavFnPlanner.maude`.

## Formal verification of the `NavFn` planner

Some properties are verified using both Dafny and manually-specified verification conditions in Maude. This is in the `src/verification` directory.

## Maude-implemented planner for Navigation 2

The Python-based ROS action server that integrates the planner into the Navigation 2 stack is in `src/maude_planner/planner_action_server.py`. Instructions for a dummy simulation are included in the file.

The Turtlebot 3 simulator can be started with the Maude planner using `src/run_simulation.sh` in an environment similar to that of `src/Dockerfile`.
