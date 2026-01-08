#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file para sistema de odometría con EKF
    
    Uso:
        ros2 launch library_opi odometry_launch.py
        ros2 launch library_opi odometry_launch.py update_rate:=100.0
    """
    
    # Nodo de odometría con EKF
    odometry_node = Node(
        package='nav_autonoma',
        executable='odom_simple',    #'ultra_robust_ekf', #dual_odometry_ekf adaptive_ekf_odometry_node odom_diff
        name='odom_simple',
        output='screen',
        emulate_tty=True,
    )
    
#    Nodo de análisis de mapa (opcional, iniciar manualmente)
    map_analysis_node = Node(
        package='nav_autonoma',
        executable='map_analysis_node',
        name='map_analysis',
        output='screen',
        emulate_tty=True,
    )
    
#     # Nodo de odometría con EKF
#     odometry_node = Node(
#         package='nav_autonoma',
#         executable='triple_odom_comparison',
#         name='triple_odom_comparison',
#         output='screen',
#         emulate_tty=True,
#     )
    
# #    Nodo de análisis de mapa (opcional, iniciar manualmente)
#     map_analysis_node = Node(
#         package='nav_autonoma',
#         executable='comparison_visualizer',
#         name='comparison_visualizer',
#         output='screen',
#         emulate_tty=True,
#     )

    control_manual=Node(
        package='nav_autonoma',
        executable='motor_simple_node',
        name='motor_simple_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyS4',
            'baudrate': 115200,
        }]
    )
    # Nodo de Socket.IO
    emisor_sio=Node(
        package='nav_autonoma',
        executable='emisor_simple',
        name='emisor_simple',
        output='screen'
    )
    # Nodo de camara
    camera=Node(
        package="nav_autonoma",
        executable="camara_ffmpeg",
        name="Camara_Node",
        output="screen"
    )

    path=Node(
        package="nav_autonoma",
        executable="path_follower_simple",
        name="path_follower_simple",
        output="screen"
    )

    rrt=Node(
        package="nav_autonoma",
        executable="rrt_planer_node",
        name="rrt_planer_node",
        output="screen"
    )

    lidar=Node(
        package="nav_autonoma",
        executable="test_taulidar_display_node",
        name="test_taulidar_display_node",
        output="screen"
    )

    return LaunchDescription([
        emisor_sio,
        odometry_node,
        control_manual,
        camera,
        map_analysis_node,  # Descomentar si quieres iniciarlo automáticamente
        path,
        rrt,
    ])
