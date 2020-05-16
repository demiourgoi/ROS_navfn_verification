from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_maude_ros',
            node_namespace='vivaMaude',
            node_executable='mauder_talker',
            node_name='maude_publisher'
        ),
        Node(
            package='pubsub_maude_ros',
            node_namespace='vivaMaude',
            node_executable='mauder_listener',
            node_name='maude_listener'
        )
    ])
