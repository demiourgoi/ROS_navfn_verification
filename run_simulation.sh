#!/bin/bash
#
# Starts the Gazebo simulator with the Maude planner
#

SOURCE_ROOT="$(dirname "$BASH_SOURCE")"

PACKAGE_DIR="$SOURCE_ROOT/maude_planner2"
PARAMS_FILE="$SOURCE_ROOT/nav2_params.yaml"

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/opt/ros/dashing/share/turtlebot3_gazebo/models

# Build and install the maude_planner package
pushd "$PACKAGE_DIR"
colcon build
source install/setup.bash
popd

# Remove the stack size limit because Maude needs it
ulimit -s unlimited

# Run the simulation
ros2 launch nav2_bringup nav2_simulation_launch.py params:="$PARAMS_FILE"
