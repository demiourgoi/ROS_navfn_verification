FROM ros:dashing
RUN apt-get update && apt-get install -y \
    ros-dashing-navigation2 \
    ros-dashing-nav2-bringup \
    ros-dashing-turtlebot3-bringup \
    ros-dashing-turtlebot3-gazebo \
    python3-pip \
    gdb \
    tmux \
    && rm -rf /var/lib/apt/lists/ \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install maude pyquaternion
