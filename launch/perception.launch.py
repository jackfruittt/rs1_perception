#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('rs1_perception')
    config_path = PathJoinSubstitution([pkg_path,'config'])

    detector_type_arg = DeclareLaunchArgument(
        'type',
        # default_value='umich',
        default_value='mit',
        description='Type of Detector Used, MIT or UMICH'
    )

    drone_count_arg = DeclareLaunchArgument(
        'drone_count',
        default_value='1',
        description='Number of Drones Available'
    )

    # apriltag_detector_node = Node(
    #     package='apriltag_detector',
    #     executable='apriltag_detector_node',
    #     parameters=[{
    #         'family': 'tag36h11',
    #         'max_hamming': 2,
    #         'quad_decimate': 1.0,
    #         'quad_sigma': 0.0,
    #         'type': LaunchConfiguration('type')
    #     }],
    # )

# Apriltag node adapted for multi-drone use
    perception_node = Node(
        package='rs1_perception',
        executable='perception_node',
        parameters=[{
            'drone_count': LaunchConfiguration('drone_count'), # This should be passed into the constructor 
            'type': LaunchConfiguration('type'),
            'family': 'tag36h11'
        }]
    )

    # Create the launch description and add the actions
    ld = LaunchDescription([
        detector_type_arg,
        drone_count_arg,
        # apriltag_detector_node,
        perception_node
    ])

    return ld