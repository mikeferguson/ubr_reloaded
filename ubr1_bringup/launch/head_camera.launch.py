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

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory

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
    # TODO: parameter rgb_camera_info_url
    # TODO: parameter depth_camera_info_url

    namespace = '/head_camera'

    # Load the driver config
    head_camera_driver_config = os.path.join(
        get_package_share_directory('ubr1_bringup'),
        'config', 'head_camera_driver.yaml'
    )

    container = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=namespace,
                    parameters=[head_camera_driver_config, ],
                    # Counting subscribers is broken when remapped
                    # remappings=[('depth/image', namespace + '/depth_registered/image_raw')],
                ),
                # Create rectified color image
                launch_ros.descriptions.ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rgb_rectify',
                    namespace=namespace,
                    # Use nearest neighbor (0)  so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', namespace + '/rgb/image_raw'),
                                ('image_rect', namespace + '/rgb/image_rect'),
                                ('camera_info', namespace + '/rgb/camera_info')],
                ),
                # Create rectified depth image
                launch_ros.descriptions.ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='depth_rectify',
                    namespace=namespace,
                    # Use nearest neighbor (0) so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', namespace + '/depth/image'),
                                ('image_rect', namespace + '/depth_registered/image_rect'),
                                ('camera_info', namespace + '/depth/camera_info')],
                ),
                # Create XYZRGB point cloud
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    namespace=namespace,
                    remappings=[('rgb/image_rect_color', namespace + '/rgb/image_rect'),
                                ('rgb/camera_info', namespace + '/rgb/camera_info'),
                                ('depth_registered/image_rect_color',
                                 namespace + '/depth_registered/image_rect'),
                                ('points', namespace + '/depth_registered/points'), ],
                ),
                # Create metric depth image
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='depth_registered_hw_metric_rect',
                    namespace=namespace,
                    remappings=[('image_raw', namespace + '/depth_registered/image_rect'),
                                ('image', namespace + '/depth_registered/image')],
                ),
                # Decimate cloud to 160x120
                # launch_ros.descriptions.ComposableNode(
                #    package='image_proc',
                #    plugin='image_proc::CropDecimateNode',
                #    name='depth_downsample',
                #    namespace=namespace,
                #    remappings=[('image_raw', namespace + '/depth_registered/image_rect'),
                #                ('image', namespace + '/depth_registered/downsample')],
                # ),
                # Downsampled XYZ point cloud (mainly for navigation)
                # launch_ros.descriptions.ComposableNode(
                #    package='depth_image_proc',
                #    plugin='depth_image_proc::PointCloudXyzNode',
                #    name='points_downsample',
                #    namespace=namespace,
                #    remappings=[('image_rect', namespace + '/depth_registered/downsample'),],
                # ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
