FROM ros:roxy
RUN apt-get update && apt-get install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-turtlebot3-bringup \
    ros-foxy-turtlebot3-gazebo \
    python3-pip \
    gdb \
    tmux \
    && rm -rf /var/lib/apt/lists/ \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install maude pyquaternion
