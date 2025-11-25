from launch import LaunchDescription
from launch_ros.actions import Node
from launch.logging import get_logger


# This function must be defined
def generate_launch_description():
    logger = get_logger("launch")
    logger.info("Using version 2...")

    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
            ),
        ]
    )
