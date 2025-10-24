from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='ID of the drone (1, 2, 3, etc.)'
        ),
        
        DeclareLaunchArgument(
            'drone_namespace',
            default_value='rs1_drone',
            description='Base namespace for the drone'
        ),
        
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug output'
        ),
        
        DeclareLaunchArgument(
            'tag_family',
            default_value='tag36h11',
            description='AprilTag family'
        ),
        
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.2',
            description='AprilTag size in meters'
        ),
        
        # Single AprilTag detector node that does everything
        Node(
            package='drone_apriltag_detector',
            executable='apriltag_detector_node',  # Updated executable name
            name=['apriltag_detector_', LaunchConfiguration('drone_id')],
            parameters=[{
                'drone_namespace': LaunchConfiguration('drone_namespace'),
                'drone_id': LaunchConfiguration('drone_id'),
                'debug': LaunchConfiguration('debug'),
                'tag_family': LaunchConfiguration('tag_family'),
                'tag_size': LaunchConfiguration('tag_size'),
            }],
            output='screen'
        )
    ])