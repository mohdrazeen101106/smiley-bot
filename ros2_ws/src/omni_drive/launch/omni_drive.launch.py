"""Launch the joystick mapper and the untouched Blitz serial sender at root."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("omni_drive"), "config", "omni_drive.yaml"
    )
    return LaunchDescription([
        # Upstream Blitz source is unchanged. Do not launch parser.py because
        # it would open the same serial port as the Pico sender.
        Node(
            package="blitz_drive",
            executable="packer.py",
            name="packer",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="omni_drive",
            executable="joy_to_velocity",
            name="joy_to_velocity",
            parameters=[config],
            output="screen",
        ),
    ])
