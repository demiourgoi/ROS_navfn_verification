#!/bin/bash
#
# Starts the Gazebo simulator with the Maude planner
#

SOURCE_ROOT="$(dirname "$BASH_SOURCE")"

PLANNER_FILE="$SOURCE_ROOT/maude_planner/planner_action_server.py"
PARAMS_FILE="$SOURCE_ROOT/nav2_params.yaml"

# Select the map: waffle, burger, waffle_pi, world
export TURTLEBOT3_MODEL=waffle

# Gazebo and nav2_bringup paths
export GAZEBO_MODEL_PATH=/opt/ros/foxy/share/turtlebot3_gazebo/models
export BRINGUP_PATH=/opt/ros/foxy/share/nav2_bringup/launch

# Update maude package
python3 -m pip install -U maude

# Remove the stack size limit because Maude needs it
ulimit -s unlimited

# Run the Maude planner
pushd "$(dirname "$PLANNER_FILE")"
python3 "$(basename "$PLANNER_FILE")" &
PLANNER_PID=$!
popd

# Run the simulation
ros2 launch nav2_bringup nav2_simulation_launch.py params:="$PARAMS_FILE"

# Stops the Maude planner
kill -9 $PLANNER_PID
