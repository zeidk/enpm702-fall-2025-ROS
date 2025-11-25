from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Declare a launch argument to enable or disable the sensor group
    enable_nav_arg = DeclareLaunchArgument(
        'enable_nav_sensors',
        default_value='false',
        description='Enable sensors needed for navigation'
    )

    # Sensor group (Lidar and Camera nodes)
    nav_sensor_group = GroupAction(
        actions=[
            Node(
                package='launch_files_demo', 
                executable='lidar_demo_node',
                output='screen'
                ),
            Node(
                package='launch_files_demo',
                executable='camera_demo_node',
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_nav_sensors'))
    )

    # Add the launch argument and sensor group to the LaunchDescription
    ld.add_action(enable_nav_arg)
    ld.add_action(nav_sensor_group)

    # Add a temperature node (always launched, not conditional)
    temperature_node = Node(
        package='launch_files_demo',
        executable='temperature_demo_node',
        output='screen'
    )

    ld.add_action(temperature_node)

    # Return the LaunchDescription object
    return ld
