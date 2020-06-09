#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import (
    LaunchDescription,
    LaunchService
)
from launch_ros.actions import Node


def generate_launch_description():
    # Load the URDF into a parameter
    bringup_dir = get_package_share_directory('ubr1_description')
    urdf_path = os.path.join(bringup_dir, 'robots', 'ubr1_robot.urdf')
    urdf = open(urdf_path).read()

    # Load the driver config
    driver_config = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'config', 'default_controllers.yaml'
    )

    return LaunchDescription([
        # Drivers
        Node(
            name='ubr_driver',
            package='ubr_drivers',
            executable='ubr_driver',
            parameters=[{'robot_description': urdf},
                        driver_config],
            output='screen',
            prefix=['xterm -e gdb --args'],
        ),
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),
        Node(
            name='base_laser_node',
            package='urg_node',
            executable='urg_node_driver',
            parameters=[{'ip_address': '10.42.0.10',
                         'angle_min': -1.54,
                         'angle_max': 1.54,
                         'laser_frame_id': 'base_laser_link'}],
            remappings=[('scan', 'base_scan')],
            output='screen',
        ),
        # TODO: head camera drivers
        # Teleop
        Node(
            name='joy',
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 1.0},],
        ),
        Node(
            name='teleop',
            package='ubr_teleop',
            executable='joystick_teleop',
            parameters=[{'autorepeat_rate': 1.0},],
            remappings=[('arm_controller_command', 'arm_controller/cartesian_twist/command'),
                        ('cmd_vel_in', 'cmd_vel'),
                        ('cmd_vel_out', 'base_controller/command')],
            output='screen',
        ),
        Node(
            name='controller_reset',
            package='ubr1_bringup',
            executable='controller_reset.py',
        )
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
