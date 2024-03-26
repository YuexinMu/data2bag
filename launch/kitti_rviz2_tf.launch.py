import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('data2bag'), 'config', 'kitti.rviz')

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_dir],
        ),

        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link velodyne".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link gray_camera_left".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link gray_camera_right".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link color_camera_left".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link color_camera_right".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link imu".split(" "),
             parameters=[]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments="0 0 0 0 0 0 base_link oxts".split(" "),
             parameters=[]),
    ])
