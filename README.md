# Maude planner for ROS Navigation 2

* The Maude implementation of the A* algorithm is in the module `ASTAR` of `src/maude/astar.maude`.
* The Python-based ROS action server that integrates the planner into the Navigation 2 stack is in `src/maude_planner/planner_action_server.py`.
* The Turtlebot 3 simulator can be started with the Maude planner using `src/run_simulation.sh` in an environment similar to that of `src/Dockerfile`.
* Auxiliary programs to profile the Maude and default planner executions are at `src/maude_planner/profiling`.
