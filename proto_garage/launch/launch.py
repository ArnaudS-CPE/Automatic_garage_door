# launch/garage_system_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for car detection
        Node(
            package='proto_garage',
            executable='car_detection',
            name='car_detection',
            output='screen'
        ),
        # Node for reading license plates
        Node(
            package='proto_garage',
            executable='read_plate',
            name='read_plate',
            output='screen'
        ),
        # Node for controlling the door (servo)
        Node(
            package='proto_garage',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
        # Node for ToF sensor management
        Node(
            package='proto_garage',
            executable='tof_node',
            name='tof_node',
            output='screen'
        ),
        # Node for checking plate validity
        Node(
            package='proto_garage',
            executable='plate_checker',
            name='plate_checker',
            output='screen'
        )
    ])
