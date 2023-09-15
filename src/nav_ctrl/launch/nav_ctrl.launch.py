from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_ctrl = Node(
        package="nav_ctrl",
        executable="nav_ctrl_node"
    )
    launch_description = LaunchDescription(
        [nav_ctrl])
    return launch_description