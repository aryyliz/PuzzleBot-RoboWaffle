from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='ClosedLoopControl',
            executable='path_generator',
            name='path_generator'
        ),

        Node(
            package='ClosedLoopControl',
            executable='closed_loop_square_ctrl',
            name='closed_loop_square_ctrl'
        ),

    ])

