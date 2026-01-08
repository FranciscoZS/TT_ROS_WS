#!/usr/bin/env python3
"""
Launch file para SLAM con TauLidar + Cartographer
Sin necesidad de URDF completo ni odometría
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Argumentos
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz2 para visualización'
    )
    
    # Ruta al archivo de configuración de Cartographer
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('nav_autonoma'),
        'config'
    ])
    
    configuration_basename = 'taulidar_2d.lua'
    
    # ==================== NODO 1: TauLidar PointCloud ====================
    taulidar_pointcloud_node = Node(
        package='nav_autonoma',
        executable='taulidar_pointcloud_node',
        name='taulidar_pointcloud',
        output='screen',
        parameters=[{
            'serial_port': 'None',  # Auto-scan
            'min_range': 100,
            'max_range': 4500,
            'publish_rate': 10.0,
            'fov_horizontal': 69.0,
            'fov_vertical': 51.0
        }]
    )
    
    # ==================== NODO 2: PointCloud → LaserScan ====================
    pointcloud_to_laserscan_node = Node(
        package='nav_autonoma',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'min_height': -0.2,
            'max_height': 0.3,
            'angle_min': -1.57,
            'angle_max': 1.57,
            'angle_increment': 0.017,
            'range_min': 0.1,
            'range_max': 4.5,
            'scan_time': 0.1
        }]
    )
    
    # ==================== NODO 3: Transformadas Estáticas ====================
    # Base_link → TauLidar_link
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'taulidar_link']
    )
    
    # Odom → Base_link (inicial, será actualizado por Cartographer)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # ==================== NODO 4: Cartographer ====================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ]
    )
    
    # Occupancy Grid Node (convierte submaps a OccupancyGrid)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05
        }],
        arguments=['-resolution', '0.05']
    )
    
    # ==================== NODO 5: RViz2 ====================
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('nav_autonoma'),
        'rviz',
        'slam_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_rviz_arg,
        taulidar_pointcloud_node,
        pointcloud_to_laserscan_node,
        static_tf_base_to_lidar,
        # static_tf_odom_to_base,  # Cartographer lo maneja
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
