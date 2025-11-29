import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='master_jo',
            executable='master_jo',
            name='master_jo',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='driving_yy',
            executable='driving_yy_node',
            name='driving_yy',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='vision_hyun',
            executable='vision_hyun_node',
            name='vision_hyun',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='turtlebot_vision',
            executable='vision_subscriber',
            name='turtlebot_vision',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='ui_test',
            executable='ui_test',
            name='ui_test',
            output='screen',
            emulate_tty=True,
        ),
    ])