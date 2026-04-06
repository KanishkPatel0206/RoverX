"""
obstacle_avoidance.launch.py
Launch the RoverX obstacle avoidance node.

Usage:
  ros2 launch my_robot_description obstacle_avoidance.launch.py
  ros2 launch my_robot_description obstacle_avoidance.launch.py obstacle_dist:=0.9
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Expose all tunable params as launch args so you can tweak without
    # editing the source file.
    args = [
        DeclareLaunchArgument('obstacle_dist',  default_value='0.7',  description='Trigger distance [m]'),
        DeclareLaunchArgument('stop_dist',      default_value='0.28', description='Emergency reverse distance [m]'),
        DeclareLaunchArgument('linear_speed',   default_value='0.22', description='Forward speed [m/s]'),
        DeclareLaunchArgument('arc_speed',      default_value='0.10', description='Speed while arc-turning [m/s]'),
        DeclareLaunchArgument('turn_speed',     default_value='0.6',  description='Rotation speed [rad/s]'),
        DeclareLaunchArgument('reverse_speed',  default_value='0.14', description='Reverse speed [m/s]'),
    ]

    node = Node(
        package='my_robot_description',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{
            'obstacle_dist':  LaunchConfiguration('obstacle_dist'),
            'stop_dist':      LaunchConfiguration('stop_dist'),
            'linear_speed':   LaunchConfiguration('linear_speed'),
            'arc_speed':      LaunchConfiguration('arc_speed'),
            'turn_speed':     LaunchConfiguration('turn_speed'),
            'reverse_speed':  LaunchConfiguration('reverse_speed'),
            'use_sim_time':   True,
        }],
        # Topics already match the bridge config in simulation.launch.py
        # /scan  → sensor_msgs/LaserScan  (Ignition → ROS2)
        # /cmd_vel → geometry_msgs/Twist  (ROS2 → Ignition)
    )

    return LaunchDescription(args + [node])