#!/usr/bin/env python3

import os
import sys

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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

    container = ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=namespace,
                    parameters=[head_camera_driver_config,],
                    # Counting subscribers is broken when remapped
                    #remappings=[('depth/image', namespace + '/depth_registered/image_raw')],
                ),
                # Create rectified color image
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rgb_rectify',
                    namespace=namespace,
                    # Use nearest neighbor (0) interpolation so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', namespace + '/rgb/image_raw'),
                                ('image_rect', namespace + '/rgb/image_rect'),
                                ('camera_info', namespace + '/rgb/camera_info')],
                ),
                # Create rectified depth image
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='depth_rectify',
                    namespace=namespace,
                    # Use nearest neighbor (0) interpolation so we don't streak across depth boundaries
                    parameters=[{'interpolation': 0}],
                    remappings=[('image', namespace + '/depth/image'),
                                ('image_rect', namespace + '/depth_registered/image_rect'),
                                ('camera_info', namespace + '/depth/camera_info')],
                ),
                # Create XYZRGB point cloud
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    namespace=namespace,
                    remappings=[('rgb/image_rect_color', namespace + '/rgb/image_rect'),
                                ('rgb/camera_info', namespace + '/rgb/camera_info'),
                                ('depth_registered/image_rect_color', namespace + '/depth_registered/image_rect'),
                                ('depth_registered/points', namespace + '/depth_registered/points')],
                ),
                # Create metric depth image
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='depth_registered_hw_metric_rect',
                    namespace=namespace,
                    remappings=[('image_raw', namespace + '/depth_registered/image_rect'),
                                ('image', namespace + '/depth_registered/image')],
                ),
                # Decimate cloud to 160x120
                #ComposableNode(
                #    package='image_proc',
                #    plugin='image_proc::CropDecimateNode',
                #    name='depth_downsample',
                #    namespace=namespace,
                #    remappings=[('image_raw', namespace + '/depth_registered/image_rect'),
                #                ('image', namespace + '/depth_registered/downsample')],
                #),
                # Downsampled XYZ point cloud (mainly for navigation)
                #ComposableNode(
                #    package='depth_image_proc',
                #    plugin='depth_image_proc::PointCloudXyzNode',
                #    name='points_downsample',
                #    namespace=namespace,
                #    remappings=[('image_rect', namespace + '/depth_registered/downsample'),],
                #),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
