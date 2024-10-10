#!/usr/bin/env python3

# Copyright (c) 2020-2023, Michael Ferguson
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

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    # Set default parameters
    bringup_dir = get_package_share_directory('ubr1_description')
    urdf_path = os.path.join(bringup_dir, 'robots', 'ubr1_robot.urdf')
    depth_camera_info_url = ''
    rgb_camera_info_url = ''
    z_offset_mm = '0'
    z_scaling = '1.0'

    # See if there is a calibration.yaml in /etc/ros/distro
    distro = os.getenv('ROS_DISTRO')
    etc_conf = os.path.join('/etc', 'ros', distro)
    calibration_yaml = os.path.join(etc_conf, 'calibration.yaml')

    if os.path.exists(calibration_yaml):
        with open(calibration_yaml) as file:
            yaml_data = yaml.load(file, Loader=yaml.FullLoader)
            urdf_path = os.path.join(etc_conf, yaml_data['urdf'])
            depth_camera_info_url = 'file://' + os.path.join(etc_conf,
                                                             yaml_data['depth_camera_info_url'])
            rgb_camera_info_url = 'file://' + os.path.join(etc_conf,
                                                           yaml_data['rgb_camera_info_url'])
            z_offset_mm = str(yaml_data['z_offset_mm'])
            z_scaling = str(yaml_data['z_scaling'])

    # Load the URDF into a parameter
    urdf = open(urdf_path).read()

    # Load the driver config
    driver_config = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'config',
        'default_controllers.yaml'
    )

    # Load the fuse config
    fuse_config = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'config',
        'fixed_lag_smoother.yaml'
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
            # This remapping is only needed when using fuse
            remappings=[('odom', 'base_controller/odom')],
            output='screen',
            # TODO use debug param
            # prefix=['xterm -e gdb --args'],
        ),
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf,
                         'publish_frequency': 100.0}],
        ),
        Node(
            name='base_laser_node',
            package='urg_node',
            executable='urg_node_driver',
            parameters=[{'ip_address': '10.42.0.10',
                         'angle_min': -1.52,
                         'angle_max': 1.52,
                         'laser_frame_id': 'base_laser_link'}],
            remappings=[('scan', 'base_scan'), ],
        ),
        Node(
            name='odom_fusion_node',
            package='fuse_optimizers',
            executable='fixed_lag_smoother_node',
            parameters=[fuse_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([head_camera_launch]),
            launch_arguments={'depth_camera_info_url': depth_camera_info_url,
                              'rgb_camera_info_url': rgb_camera_info_url,
                              'z_offset_mm': z_offset_mm,
                              'z_scaling': z_scaling}.items()
        ),
        # Teleop
        Node(
            name='joy',
            package='joy_linux',
            executable='joy_linux_node',
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
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
