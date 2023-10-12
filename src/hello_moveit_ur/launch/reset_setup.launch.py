#This launch file starts all the neccesary nodes and launches them
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

    init_node = Node(
        package="hello_moveit_ur",
        executable="reset_node",
        name="reset_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )
    vision_node = Node(
        package="hello_moveit_ur",
        executable="vision_node",
        name="vision_node",
        output="screen",
    )

    vision_test = Node(
        package="hello_moveit_ur",
        executable="vision_test",
        name="vision_test",
        output="screen",
    )

    vision_node_client_moveit = Node(
        package="hello_moveit_ur",
        executable="vision_node_client_moveit",
        name="vision_node_client_moveit",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    robot_cam_tf= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '-0.15', '--z', '0.7', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )

    world_cam_tf= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '-0.15', '--z', '0.7', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'camera_depth_optical_frame']
    )

    world_cam_tf2= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '-0.15', '--z', '0.7', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'camera_color_optical_frame']
    )

    world_robot_tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #will be changed later with real values
        arguments = ['--x', '0.0', '--y', '-0.0', '--z', '0.00', '--yaw', '-0.0', '--pitch', '-0.0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
    )

    return launch.LaunchDescription([robot_cam_tf,world_cam_tf,world_cam_tf2,world_robot_tf,vision_node,vision_test,vision_node_client_moveit ])
