## Verification of [ROS Navigation 2](https://navigation.ros.org/)'s NavFn planner using [Maude](http://maude.cs.illinois.edu) and [Dafny](https://dafny.org/)

The Maude implementations of the NavFn algorithm are in the `src/maude` directory. There are various versions:

* NavFn<sub>A*</sub> is a textbook implementation of the A* algorithm. It can be executed with the [`a*`](src/maude/astar.maude#L158) operator of the `ASTAR` module in `astar.maude`.
* NavFn<sub>pot</sub> calculates a potential function imitating the [`NavFn` planner](https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner) of Navigation 2, and then computes the path from the origin to the goal by always taking the neighbor with the lowest potential as the next position. This is the [`computePath`](src/maude/common.maude#L329) function of the `BASIC-TRAVERSE` module in `common.maude`.
* NavFn<sub>ROS</sub> imitates the complete `NavFN` implementation, also in the second phase where the path is computed. It is available in the [`a*`](src/maude/astar_no_turnNavFnPlanner.maude#L33) function of the `ASTAR` module in `astar_no_turnNavFnPlanner.maude`.

The Dafny implementation of the NavFn planner are in the `src/dafny` directory.

### Formal verification of the `NavFn` planner

Some properties are verified using both Dafny (see [`src/dafny`](src/dafny)) and manually-specified verification conditions in Maude (see [`src/maude/verification`](src/maude/verification/)).

### Maude-implemented planner for Navigation 2

The Python-based ROS action server that integrates the planner into the Navigation 2 stack is in the [`src/maude/integration`](src/maude/integration) directory. The main file, [`planner_action_server.py`](src/maude/integration/planner_action_server.py), includes instructions for a dummy simulation.

The Turtlebot 3 simulator can be started with the Maude planner using [`run_simulation.sh`](src/maude/integration/run_simulation.sh) in an environment similar to that of [`Dockerfile`](src/maude/integration/Dockerfile).
