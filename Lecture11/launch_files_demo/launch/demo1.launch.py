from launch import LaunchDescription
from launch_ros.actions import Node
from launch.logging import get_logger


# This function must be defined
def generate_launch_description():
    logger = get_logger("launch")
    logger.info("Using version 1...")

    ld = LaunchDescription()
    talker = Node(package="demo_nodes_cpp", executable="talker")
    listener = Node(package="demo_nodes_cpp", executable="listener")

    ld.add_action(talker)
    ld.add_action(listener)

    return ld
