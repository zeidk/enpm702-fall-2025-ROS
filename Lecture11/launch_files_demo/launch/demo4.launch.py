from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    # Declare argument
    declare_talker_arg = DeclareLaunchArgument(
        "talker_arg", default_value="false", description="Enable talker node"
    )

    # Conditionally launched node
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        condition=IfCondition(LaunchConfiguration("talker_arg")),
    )

    ld.add_action(declare_talker_arg)
    ld.add_action(talker_node)
    return ld
