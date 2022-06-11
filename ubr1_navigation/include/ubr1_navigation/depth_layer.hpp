/*
 * Copyright (c) 2020, Michael Ferguson
 * Copyright (c) 2015-2016, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Anuj Pasricha, Michael Ferguson

#ifndef UBR1_NAVIGATION__DEPTH_LAYER_HPP_
#define UBR1_NAVIGATION__DEPTH_LAYER_HPP_

#include <mutex>

#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/message_filter.h"
//#include <message_filters/subscriber.h>

#include <opencv2/rgbd.hpp>
using cv::rgbd::DepthCleaner;
using cv::rgbd::RgbdNormals;
using cv::rgbd::RgbdPlane;
using cv::rgbd::depthTo3d;

namespace nav2_costmap_2d
{

/**
 * @class DepthLayer
 * @brief A costmap layer that extracts ground plane and clears it.
 */
class DepthLayer : public VoxelLayer
{
public:
  /**
   * @brief Constructor
   */
  DepthLayer();

  /**
   * @brief Destructor for the depth costmap layer
   */
  virtual ~DepthLayer();

  /**
   * @brief Initialization function for the DepthLayer
   */
  virtual void onInitialize();

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void depthImageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::shared_ptr<nav2_costmap_2d::ObservationBuffer> marking_buf_;
  std::shared_ptr<nav2_costmap_2d::ObservationBuffer> clearing_buf_;

  // should we publish the marking/clearing observations
  bool publish_observations_;

  // distance away from ground plane at which
  // something is considered an obstacle
  double observations_threshold_;

  // if finding ground plane, limit the tilt
  // with respect to base_link frame
  double ground_threshold_;

  // should NANs be treated as +inf and used for clearing
  bool clear_nans_;

  // skipping of potentially noisy rays near the edge of the image
  int skip_rays_bottom_;
  int skip_rays_top_;
  int skip_rays_left_;
  int skip_rays_right_;

  // should skipped edge rays be used for clearing?
  bool clear_with_skipped_rays_;

  // retrieves depth image from head_camera
  // used to fit ground plane to
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image,
                                              rclcpp_lifecycle::LifecycleNode>> depth_image_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::Image>> depth_image_filter_;

  // retrieves camera matrix for head_camera
  // used in calculating ground plane
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // publishes clearing observations
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;

  // publishes marking observations
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr marking_pub_;

  // camera intrinsics
  std::mutex mutex_K_;
  cv::Mat K_;

  // clean the depth image
  cv::Ptr<DepthCleaner> depth_cleaner_;

  // depth image estimation
  cv::Ptr<RgbdNormals> normals_estimator_;
  cv::Ptr<RgbdPlane> plane_estimator_;
};

}  // namespace nav2_costmap_2d

#endif  // UBR1_NAVIGATION__DEPTH_LAYER_HPP
