from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robot_description_pkg'  # your actual package name

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory(pkg_name),
                'config',
                'slam_toolbox_params.yaml'
            )]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory(pkg_name),
                'config',
                'slam_config.rviz'  # optional, only if you want custom view
            )],
            parameters=[{'use_sim_time': True}]
        )
    ])
