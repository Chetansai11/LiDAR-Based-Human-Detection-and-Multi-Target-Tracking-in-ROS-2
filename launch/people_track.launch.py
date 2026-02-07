#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Optional args if you still want them
    ld.add_action(DeclareLaunchArgument('bag_in', default_value=''))
    ld.add_action(DeclareLaunchArgument('bag_out', default_value='person_tracker_output'))

    # Only our nodes
    detector_node = Node(
        package='person_tracker',
        executable='detector',
        name='lidar_detector',
        output='screen',
    )

    tracker_node = Node(
        package='person_tracker',
        executable='tracker',
        name='person_tracker',
        output='screen',
    )

    ld.add_action(detector_node)
    ld.add_action(tracker_node)

    return ld
