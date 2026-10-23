from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('r2'),
            Node(
                package='blitz_drive',
                executable='packer.py',
                name='packer',
                output='screen'
            ),
            Node(
                package='blitz_drive',
                executable='parser.py',
                name='parser',
                output='screen'
            )
        ])
    ])
