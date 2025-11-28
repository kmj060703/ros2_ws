from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='master_jo',
            executable='imu_jo',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='master_jo',
            executable='master_jo',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='master_jo',
            executable='psd_jo',
            output='screen',
            emulate_tty=True,
        ),
    ])
