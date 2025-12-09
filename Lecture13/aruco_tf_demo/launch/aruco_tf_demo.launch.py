"""
Launch file for ArUco detection with ROSbot in Gazebo.

This launch file:
1. Launches the ROSbot Gazebo simulation
2. Starts the ros_gz_bridge for camera topics (fixes the ROSbot bridge issue)
3. Launches the ArUco detector node with correct parameters

Usage:
    ros2 launch aruco_tf_demo aruco_rosbot.launch.py

With custom parameters:
    ros2 launch aruco_tf_demo aruco_rosbot.launch.py marker_size:=0.1 robot_model:=rosbot_xl
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    
    # ROSbot simulation arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='rosbot',
        description='Robot model: rosbot or rosbot_xl'
    )

    # ArUco detection arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.124',
        description='Size of ArUco markers in meters'
    )

    dictionary_id_arg = DeclareLaunchArgument(
        'dictionary_id',
        default_value='0',
        description='ArUco dictionary ID (0=DICT_4X4_50, 10=DICT_6X6_250)'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='oak_rgb_camera_optical_frame',
        description='Camera optical frame for TF broadcasts (must be optical frame for correct orientation)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/oak/rgb/color',
        description='Camera image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/oak/rgb/camera_info',
        description='Camera info topic'
    )

    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Publish annotated debug image'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Reference frame for marker poses in odom frame'
    )

    publish_odom_poses_arg = DeclareLaunchArgument(
        'publish_odom_poses',
        default_value='true',
        description='Publish marker poses transformed to odom frame'
    )

    use_kdl_arg = DeclareLaunchArgument(
        'use_kdl',
        default_value='false',
        description='Use KDL for manual transform composition instead of TF2 listener'
    )

    # ==========================================================================
    # Include ROSbot Gazebo Simulation
    # ==========================================================================
    rosbot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbot_gazebo'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
        }.items()
    )

    # ==========================================================================
    # ROS-Gazebo Bridge for Camera Topics
    # Fixes the issue where ROSbot's default bridge doesn't forward camera data
    # Delayed to ensure Gazebo is running
    # ==========================================================================
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='aruco_camera_bridge',
        arguments=[
            '/oak/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/oak/rgb/color@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    # Delay camera bridge to ensure Gazebo simulation is ready
    delayed_camera_bridge = TimerAction(
        period=5.0,
        actions=[camera_bridge_node]
    )

    # ==========================================================================
    # ArUco Detector Node
    # Delayed start to ensure bridge is ready
    # ==========================================================================
    aruco_detector_node = Node(
        package='aruco_tf_demo',
        executable='aruco_detector_node',
        name='aruco_detector',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'dictionary_id': LaunchConfiguration('dictionary_id'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'publish_odom_poses': LaunchConfiguration('publish_odom_poses'),
            'use_kdl': LaunchConfiguration('use_kdl'),
        }],
        output='screen'
    )

    # Delay ArUco node start to ensure simulation and bridge are ready
    delayed_aruco_node = TimerAction(
        period=8.0,
        actions=[aruco_detector_node]
    )

    # ==========================================================================
    # Launch Description
    # ==========================================================================
    return LaunchDescription([
        # Arguments
        robot_model_arg,
        marker_size_arg,
        dictionary_id_arg,
        camera_frame_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        publish_debug_image_arg,
        odom_frame_arg,
        publish_odom_poses_arg,
        use_kdl_arg,

        # Launch ROSbot simulation first
        rosbot_gazebo_launch,

        # Then start camera bridge (delayed 5s)
        delayed_camera_bridge,

        # Finally start ArUco detector (delayed 8s)
        delayed_aruco_node,
    ])