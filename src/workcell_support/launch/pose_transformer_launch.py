import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='world',
            description='Base frame for transforming poses'
        ),
        Node(
            package='workcell_core',  # Replace with the actual package name
            executable='vision_node',  # Replace with the executable name of your Vision node',  # Replace with the executable name of your Localizer node
            name='vision_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/aruco_poses', '/aruco_poses'),  # Adjust remappings as needed
            ]
        ),
        Node(
            package='workcell_core',  # Replace with the actual package name
            executable='pose_transformer_node',  # Replace with the executable name of your PoseTransformer node
            name='pose_transformer_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'base_frame': LaunchConfiguration('world')}  # Pass the base_frame as a parameter
            ],
            remappings=[
                ('/aruco_poses', '/aruco_poses'),  # Adjust remappings as needed
            ]
        )

        
    ])
