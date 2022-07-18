## Fix for the collision problems of the original NavFn planner

The file `avoidObstacles.patch` is a patch to the NavFn planner implementation with which
no step of the path is planned inside an obstacle cell.

It should be applied in the root directory of the [ros-planning/navigation2](https://github.com/ros-planning/navigation2)
repository. The NavFn planner can then be built as usual.
