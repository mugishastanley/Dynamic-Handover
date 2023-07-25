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
    demo_node = Node(
        package="hello_moveit_ur",
        executable="hello_moveit_ur",
        name="hello_moveit_ur",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([demo_node])
