import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_sjtu_drone_bringup_path = get_package_share_directory('drone_exploration')
    rtabmap_launch_path = get_package_share_directory('drone_exploration')

    default_db_path = os.path.expanduser('~/ros2_ws/src/maps/drone_map.db')

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=default_db_path,
        description='Path to the RTAB-Map database to save to or resume from'
    )

    delete_db_on_start = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Set to true to start a fresh map, false to resume/continue from database path'
    )

    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_path, 'launch', 'rtabmap_drone.launch.py')
        ),
        launch_arguments={
            'database_path': LaunchConfiguration('database_path'),
            'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
        }.items()
    )

    delayed_rtabmap = TimerAction(
        period=90.0,
        actions=[rtabmap_include]
    )

    # Publishes/consumes the TF-filtered laser cloud. Needs the TF tree
    # (static laser_link + dynamic odom broadcaster) up and stable first,
    # so it's staggered the same way the rest of the sensor stack is.
    laser_tf_node = Node(
        package='drone_exploration',
        executable='laser_tf_filtered_node',
        name='laser_tf_filtered_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_laser_tf = TimerAction(
        period=8.0,
        actions=[laser_tf_node]
    )

    return LaunchDescription([
        database_path_arg,
        delete_db_on_start,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_sjtu_drone_bringup_path, 'launch', 'my_sjtu_drone_bringup.launch.py')
            )
        ),

        delayed_laser_tf,
        delayed_rtabmap,
    ])