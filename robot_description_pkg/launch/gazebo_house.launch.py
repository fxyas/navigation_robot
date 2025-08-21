from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = 'robot_description_pkg' 

    #  Get the path to your world file
    world_path = os.path.join(
        get_package_share_directory(pkg_name),
        'worlds',
        'turtlebot3_house.world'  
    )

    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'amr.urdf'
    )

    # Start Gazebo with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'#'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items()  
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_entity
    ])
