#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav_autonoma",
            executable="dual_odometry_node",
            name="dual_odometry_node",
            output="screen"
        ),
        # Nodo de mpu
        Node(
            package="nav_autonoma",
            executable="map_saver_node",
            name="map_saver_node",
            output="screen"
        ),
    ])