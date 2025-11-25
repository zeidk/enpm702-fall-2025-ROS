from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


# This function must be defined
def generate_launch_description():
    ld = LaunchDescription()

    # Include talker_listener.launch.py from demo_nodes_cpp
    talker_listener_version_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("demo_nodes_cpp"),
                        "launch",
                        "topics",
                        "talker_listener_launch.py",
                    ]
                )
            ]
        )
    )

    # Include talker_listener.launch.py from demo_nodes_cpp
    talker_listener_version_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("demo_nodes_cpp")
            + "/launch/topics/talker_listener_launch.py"
        )
    )

    ld.add_action(talker_listener_version_1)
    # ld.add_action(talker_listener_version_2)

    return ld
