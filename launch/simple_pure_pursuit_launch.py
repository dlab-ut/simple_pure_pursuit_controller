from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_pure_pursuit_controller',
            executable='simple_pure_pursuit_controller',
            name='simple_pure_pursuit_controller',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel_parallel'),
                ('/scan', '/integrated_mid'),
            ],
            parameters=[
                {
                    'lookahead_distance': 1.0,
                    'deceleration_distance': 1.0,
                    'stop_distance': 0.2,
                    'max_linear_velocity': 0.5,
                    'max_angular_velocity': 0.4,
                    'obstacle_distance_threshold': 1.0,
                    'obstacle_width_threshold': 0.4,
                }
            ]
        ),
    ])
