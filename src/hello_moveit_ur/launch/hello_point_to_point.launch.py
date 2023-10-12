import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from moveit_commons import get_robot_description, get_robot_description_semantic

def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    demo_node = Node(
        package="hello_moveit_ur",
        executable="point_to_point",
        name="point_to_point",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    world_cam_tf= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '-0.2', '--z', '1.55', '--yaw', '-0.0', '--pitch', '-0.0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'camera_color_optical_frame']
    )
    world_robot_tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '-0.0', '--z', '0.85', '--yaw', '-0.0', '--pitch', '-0.0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
    )
    

    return launch.LaunchDescription([world_cam_tf,demo_node,robot_state_publisher])
