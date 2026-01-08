#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de motores
        Node(
            package='nav_autonoma',
            executable='motor_simple_node',
            name='motor_simple_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
            }]
        ),
        # Nodo de Socket.IO
        Node(
            package='nav_autonoma',
            executable='emisor_simple',
            name='emisor_simple',
            output='screen'
        ),
        # Nodo de camara
        Node(
            package="nav_autonoma",
            executable="camara_ffmpeg",
            name="Camara_Node",
            output="screen"
        ),
        # Nodo de mpu
        # Node(
        #     package="nav_autonoma",
        #     executable="test_mpu_node",
        #     name="test_mpu_node",
        #     arguments=["--ros-args", "--log-level", "FATAL"],
        #     #output="screen"
        # ),
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