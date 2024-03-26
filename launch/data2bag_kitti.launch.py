import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    yaml_config_dir = os.path.join(get_package_share_directory('data2bag'), 'config', 'kitti.yaml')

    save_bag_path = "/home/myx/develop/Swarm-SLAM/src/cslam_experiments/data/KITTI00/kitti-1"

    return LaunchDescription([
        DeclareLaunchArgument('if_bag_record', default_value='False', description='Whether to record bag or not'),
        DeclareLaunchArgument('if_rviz2_open', default_value='True', description='Whether to open rviz2 or not'),

        Node(
            package='data2bag',
            executable='data2bag_kitti',
            name='data2bag_kitti',
            output='screen',
            parameters=[yaml_config_dir],
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', save_bag_path, "/kitti/point_cloud", "/kitti/image/gray/left",
                 "/kitti/image/gray/right", "/kitti/image/color/left", "/kitti/image/color/right", "/kitti/imu",
                 "/kitti/nav_sat_fix", "/kitti/marker_array"],
            output='screen',
            condition=IfCondition(LaunchConfiguration("if_bag_record"))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("data2bag"),
                             "launch", "kitti_rviz2_tf.launch.py")),
            condition=IfCondition(LaunchConfiguration("if_rviz2_open"))
        ),
    ])
