from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camara_node = Node(
        package="nav_autonoma",
        executable="camara_ffmpeg",
        name="Camara_Node",
        output="screen"
    )
    
    return LaunchDescription([
        camara_node
    ])