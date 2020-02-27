from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_ros',
            node_namespace='noimporta',
            node_executable='talker',
            node_name='maude_publisher'
        ),
        Node(
            package='pubsub_ros',
            node_namespace='noimporta',
            node_executable='listener',
            node_name='maude_listener'
        )
    ])
