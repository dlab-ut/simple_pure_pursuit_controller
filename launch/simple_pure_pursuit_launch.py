from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_pure_pursuit_controller',
            executable='simple_pure_pursuit_controller',
            name='simple_pure_pursuit_controller',
            output='screen',
            parameters=[
                {
                    'lookahead_distance': 0.5,
                    'deceleration_distance': 1.0,
                    'stop_distance': 0.25,
                    'max_linear_velocity': 0.3,
                    'max_angular_velocity': 1.5,
                    'obstacle_distance_threshold': 0.5,
                    'obstacle_width_threshold': 0.4,
                }
            ]
        ),
    ])
