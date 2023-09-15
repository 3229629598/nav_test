from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tf_broadcaster = Node(
        package="tf_broadcaster",
        executable="tf_broadcaster_node"
    )
    launch_description = LaunchDescription(
        [tf_broadcaster])
    return launch_description