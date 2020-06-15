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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

"""
Topic map:

  /head_camera
    /depth
      /camera_info
      /image_raw
    /depth_registered
      /image_raw -> DEPTH_RECTIFY -> /image_rect
                 -> HW_METRIC -> /image
    /ir
      /camera_info
      /image
    /rgb
      /camera_info -> RGB_RECTIFY
      /image_raw -> RGB_RECTIFY -> /rgb/image_rect
    /projector
      /camera_info
"""


def generate_launch_description():

    return LaunchDescription([
        # Arguments first
        DeclareLaunchArgument(
            'namespace', default_value='/head_camera'
        ),
        DeclareLaunchArgument(
            'rgb_frame_id', default_value='head_camera_rgb_optical_frame'
        ),
        DeclareLaunchArgument(
            'depth_frame_id', default_value='head_camera_rgb_depth_optical_frame'
        ),
        ComposableNodeContainer(
            name='container',
            namespace=LaunchConfiguration('namespace'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{'rgb_frame_id':
                                 LaunchConfiguration('rgb_frame_id'),
                                 'depth_frame_id':
                                 LaunchConfiguration('depth_frame_id'),
                                 'depth_registration': True,
                                 'data_skip': 1,  # throttle to 15hz
                                 'use_device_time': False}, ],
                    remappings=[('depth/image', 'depth_registered/image_raw')],
                ),
                # Create rectified color image
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rgb_rectify',
                    namespace=LaunchConfiguration('namespace'),
                    # Use nearest neighbor (0)  so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', 'rgb/image_raw'),
                                ('image_rect', 'rgb/image_rect'),
                                ('camera_info', 'rgb/camera_info')],
                ),
                # Create rectified depth image
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='depth_rectify',
                    namespace=LaunchConfiguration('namespace'),
                    # Use nearest neighbor (0) so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', 'depth_registered/image_raw'),
                                ('image_rect', 'depth_registered/image_rect'),
                                ('camera_info', 'depth/camera_info')],
                ),
                # Create XYZRGB point cloud
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    namespace=LaunchConfiguration('namespace'),
                    remappings=[('rgb/image_rect_color', 'rgb/image_rect'),
                                ('rgb/camera_info', 'rgb/camera_info'),
                                ('depth_registered/image_rect', 'depth_registered/image_rect'),
                                ('points', 'depth_registered/points'), ],
                ),
                # Create metric depth image
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='depth_registered_hw_metric_rect',
                    namespace=LaunchConfiguration('namespace'),
                    remappings=[('image_raw', 'depth_registered/image_rect'),
                                ('image', 'depth_registered/image')],
                ),
                # Decimate cloud to 160x120
                # ComposableNode(
                #    package='image_proc',
                #    plugin='image_proc::CropDecimateNode',
                #    name='depth_downsample',
                #    namespace=LaunchConfiguration('namespace'),
                #    remappings=[('image_raw', 'depth_registered/image_rect'),
                #                ('image', 'depth_registered/downsample')],
                # ),
                # Downsampled XYZ point cloud (mainly for navigation)
                # ComposableNode(
                #    package='depth_image_proc',
                #    plugin='depth_image_proc::PointCloudXyzNode',
                #    name='points_downsample',
                #    namespace=LaunchConfiguration('namespace'),
                #    remappings=[('image_rect', 'depth_registered/downsample'),],
                # ),
            ],
            output='screen',
        ),
    ])
