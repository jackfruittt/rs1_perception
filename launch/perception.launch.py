#!/usr/bin/env python3
# -*-python-*---------------------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
# Modifications Copyright 2025 Jackson Russell and contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
@file perception.launch.py
@brief Dynamic perception node launcher for drone swarm operations

This file contains modified code originally from apriltag_detector.
Original copyright retained above with modifications noted.

@author Jackson Russell
ADD AUTHORS HERE AND BELOW
@date Oct-2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Generate launch description for dynamic perception node
    
    Supports dynamic drone namespace configuration for multi-drone operations.
    Follows the same pattern as the drone controller node for consistency.
    """
    pkg_path = FindPackageShare('rs1_perception')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Declare launch arguments for dynamic configuration
    drone_namespace_arg = DeclareLaunchArgument(
        'drone_namespace',
        default_value='rs1_drone',
        description='ROS namespace for the drone (e.g., rs1_drone_1, rs1_drone_2)'
    )
    
    detector_type_arg = DeclareLaunchArgument(
        'detector_type',
        default_value='umich',
        description='AprilTag detector type: umich or mit'
    )
    
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tf36h11',
        description='AprilTag family (tf36h11, tf25h9, etc.)'
    )
    
    image_transport_arg = DeclareLaunchArgument(
        'image_transport',
        default_value='raw',
        description='Image transport type (raw, compressed, etc.)'
    )
    
    focal_length_arg = DeclareLaunchArgument(
        'front_camera_focal_length',
        default_value='185.7',
        description='Front camera focal length in pixels'
    )

    # Create perception node with dynamic namespace support
    perception_node = Node(
        package='rs1_perception',
        executable='perception_node',
        name='perception_node',
        namespace=LaunchConfiguration('drone_namespace'),
        parameters=[{
            'drone_namespace': LaunchConfiguration('drone_namespace'),
            'detector_type': LaunchConfiguration('detector_type'),
            'tag_family': LaunchConfiguration('tag_family'),
            'image_transport': LaunchConfiguration('image_transport'),
            'front_camera_focal_length': LaunchConfiguration('front_camera_focal_length'),
            'april_tag_size': 1.0,
            'position_tolerance': 0.5,
            'image_qos_profile': 'default',
            'decimate_factor': 1.0,
            'blur': 0.0,
            'num_threads': 1,
            'max_allowed_hamming_distance': 0,
            'black_border_width': 1
        }],
        output='screen',
        emulate_tty=True
    )

    # Create the launch description and add all components
    ld = LaunchDescription([
        drone_namespace_arg,
        detector_type_arg,
        tag_family_arg,
        image_transport_arg,
        focal_length_arg,
        perception_node
    ])

    return ld