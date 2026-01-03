from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'nav_autonoma'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
        glob('config/*.lua')),
        # Configuraciones de RViz
        (os.path.join('share', package_name, 'rviz'),
        glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Nodos principales del sistema
            'odometry_node = nav_autonoma.odometry_node:main',
            'motor_controller_node = nav_autonoma.motor_controller_node:main',
            
            # Nodos de prueba individuales
            'test_encoder_node = nav_autonoma.test_encoder_node:main',
            'test_mpu_node = nav_autonoma.test_mpu_node:main',
            'test_sdc40_node = nav_autonoma.test_sdc40_node:main',
            'test_led_node = nav_autonoma.test_led_node:main',
            'test_motor_node = nav_autonoma.test_motor_node:main',
            'test_pwm_node = nav_autonoma.test_pwm_node:main',
            'test_move_node = nav_autonoma.test_move_node:main',
            'test_audio_node = nav_autonoma.test_audio_node:main',
            'test_camera_node = nav_autonoma.test_camera_node:main',
            'test_camera_display_node = nav_autonoma.test_camera_display:main',
            'test_taulidar_node = nav_autonoma.test_taulidar_node:main',
            'test_taulidar_display_node = nav_autonoma.test_taulidar_display_node:main',
            'test_ads1115_node = nav_autonoma.test_ads1115_node:main',
            'test_yolo_detection_node = nav_autonoma.test_yolo_detection_node:main',
            'test_yolo_display_node = nav_autonoma.test_yolo_display_node:main',
            'yolo_audio_node = nav_autonoma.yolo_audio_node:main',
            'taulidar_pointcloud_node = nav_autonoma.taulidar_pointcloud_node:main',
            'pointcloud_to_laserscan_node = nav_autonoma.pointcloud_to_laserscan_node:main',
            'emisor_node = nav_autonoma.GUI_Orange.emisor_node:main',
            'camara_ffmpeg = nav_autonoma.GUI_Orange.camara_ffmpeg:main',
            'motor_simple_node = nav_autonoma.motor_simple_node:main',
            'emisor_simple = nav_autonoma.GUI_Orange.emisor_simple:main',
            'AudioNode = nav_autonoma.AudioNode:main',
            'dual_odometry_node = nav_autonoma.dual_odometry_node:main',
            'map_saver_node = nav_autonoma.map_saver_node:main',
            'dual_odometry_ekf = nav_autonoma.dual_odometry_ekf:main',
            'map_analysis_node = nav_autonoma.map_analysis_node:main',
            'diagnostic_node = nav_autonoma.diagnostic_node:main',
            'adaptive_ekf_odometry_node = nav_autonoma.adaptive_ekf_odometry_node:main',
            'comparison_visualizer = nav_autonoma.comparison_visualizer:main',
            'triple_odom_comparison = nav_autonoma.triple_odom_comparison:main',
            'ultra_robust_ekf = nav_autonoma.ultra_robust_ekf:main',
            'path_follower_simple = nav_autonoma.path_follower_simple:main',
            'odom_diff = nav_autonoma.odom_diff:main',
            'rrt_planer_node = nav_autonoma.rrt_planer_node:main'
        ],
    },
)
