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

    world_mycam_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.2689', '--y', '-0.197', '--z', '-0.826', '--yaw', '-0.678', '--pitch', '-0.5', '--roll', '1.475', '--frame-id', 'world', '--child-frame-id', 'mycam']
    )

    return launch.LaunchDescription([world_mycam_tf_node,demo_node])
