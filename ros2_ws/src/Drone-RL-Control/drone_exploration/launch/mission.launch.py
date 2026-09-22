from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_exploration',
            executable='exploration_drone_executable',
            output='screen'
        )
    ])