#!/usr/bin/env python3

# Copyright (c) 2020, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Load the URDF into a parameter
    bringup_dir = get_package_share_directory('ubr1_description')
    urdf_path = os.path.join(bringup_dir, 'robots', 'ubr1_robot.urdf')
    urdf = open(urdf_path).read()

    # Load the driver config
    driver_config = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'config',
        'default_controllers.yaml'
    )

    # Get path to the head camera launch file
    head_camera_launch = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'launch',
        'head_camera.launch.py'
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
            # TODO use debug param
            #prefix=['xterm -e gdb --args'],
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
            remappings=[('scan', 'base_scan'), ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([head_camera_launch])
        ),
        # Teleop
        Node(
            name='joy',
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 1.0}, ],
        ),
        Node(
            name='teleop',
            package='ubr_teleop',
            executable='joystick_teleop',
            parameters=[{'autorepeat_rate': 1.0}, ],
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
