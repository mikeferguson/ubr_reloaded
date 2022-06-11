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

#include <limits>

#include "ubr1_navigation/depth_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DepthLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("depth_layer");

DepthLayer::DepthLayer()
{
}

void DepthLayer::onInitialize()
{
  VoxelLayer::onInitialize();

  declareParameter("publish_observations", rclcpp::ParameterValue(false));
  declareParameter("observations_separation_threshold", rclcpp::ParameterValue(0.06));

  // Detect the ground plane
  declareParameter("ground_orientation_threshold", rclcpp::ParameterValue(0.9));

  // Should NANs be used as clearing observations?
  declareParameter("clear_nans", rclcpp::ParameterValue(false));

  // Observation range values for both marking and clearing
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("min_clearing_height", rclcpp::ParameterValue(-std::numeric_limits<double>::infinity()));
  declareParameter("max_clearing_height", rclcpp::ParameterValue(std::numeric_limits<double>::infinity()));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.0));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(2.5));
  declareParameter("raytrace_min_range", rclcpp::ParameterValue(0.0));
  declareParameter("raytrace_max_range", rclcpp::ParameterValue(3.0));

  // Skipping of potentially noisy rays near the edge of the image
  declareParameter("skip_rays_bottom", rclcpp::ParameterValue(10));
  declareParameter("skip_rays_top", rclcpp::ParameterValue(10));
  declareParameter("skip_rays_left", rclcpp::ParameterValue(10));
  declareParameter("skip_rays_right", rclcpp::ParameterValue(10));

  // Should skipped edge rays be used for clearing?
  declareParameter("clear_with_skipped_rays", rclcpp::ParameterValue(false));

  // How long should observations persist?
  declareParameter("observation_persistence", rclcpp::ParameterValue(0.0));

  // How often should we expect to get sensor updates?
  declareParameter("expected_update_rate", rclcpp::ParameterValue(0.0));

  // How long to wait for transforms to be available?
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.5));

  // Topic names
  declareParameter("depth_topic", rclcpp::ParameterValue(std::string("/head_camera/depth_downsample/image_raw")));
  declareParameter("info_topic", rclcpp::ParameterValue(std::string("/head_camera/depth_downsample/camera_info")));

  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Get parameters
  node->get_parameter(name_ + ".publish_observations", publish_observations_);
  node->get_parameter(name_ + ".observations_separation_threshold", observations_threshold_);
  node->get_parameter(name_ + ".ground_orientation_threshold", ground_threshold_);
  node->get_parameter(name_ + ".clear_nans", clear_nans_);
  double min_obstacle_height, max_obstacle_height, min_clearing_height, max_clearing_height;
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height);
  node->get_parameter(name_ + ".min_clearing_height", min_clearing_height);
  node->get_parameter(name_ + ".max_clearing_height", max_clearing_height);
  node->get_parameter(name_ + ".skip_rays_bottom", skip_rays_bottom_);
  node->get_parameter(name_ + ".skip_rays_top",    skip_rays_top_);
  node->get_parameter(name_ + ".skip_rays_left",   skip_rays_left_);
  node->get_parameter(name_ + ".skip_rays_right",  skip_rays_right_);
  node->get_parameter(name_ + ".clear_with_skipped_rays", clear_with_skipped_rays_);
  double observation_keep_time;
  node->get_parameter(name_ + ".observation_persistence", observation_keep_time);
  double expected_update_rate;
  node->get_parameter(name_ + ".expected_update_rate", expected_update_rate);
  double transform_tolerance;
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  double obstacle_min_range, obstacle_max_range, raytrace_min_range, raytrace_max_range;
  node->get_parameter(name_ + ".obstacle_min_range", obstacle_min_range);
  node->get_parameter(name_ + ".obstacle_max_range", obstacle_max_range);
  node->get_parameter(name_ + ".raytrace_min_range", raytrace_min_range);
  node->get_parameter(name_ + ".raytrace_max_range", raytrace_max_range);

  std::string sensor_frame = "";

  marking_buf_ = std::make_shared<nav2_costmap_2d::ObservationBuffer>(
    node, name_, observation_keep_time, expected_update_rate,
    min_obstacle_height, max_obstacle_height, obstacle_max_range,
    obstacle_min_range, raytrace_max_range, raytrace_min_range,
    *tf_, global_frame_, sensor_frame, tf2::durationFromSec(transform_tolerance));
  marking_buffers_.push_back(marking_buf_);
  observation_buffers_.push_back(marking_buf_);

  clearing_buf_ = std::make_shared<nav2_costmap_2d::ObservationBuffer>(
    node_, name_, observation_keep_time, expected_update_rate,
    min_clearing_height, max_clearing_height, obstacle_max_range,
    obstacle_min_range, raytrace_max_range, raytrace_min_range,
    *tf_, global_frame_, sensor_frame, tf2::durationFromSec(transform_tolerance));
  clearing_buffers_.push_back(clearing_buf_);
  observation_buffers_.push_back(clearing_buf_);

  rclcpp::QoS points_qos(1);
  points_qos.best_effort();
  if (publish_observations_)
  {
    clearing_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("clearing_obs", points_qos);
    marking_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("marking_obs", points_qos);
    clearing_pub_->on_activate();
    marking_pub_->on_activate();
  }

  // Subscribe to camera/info topics
  std::string camera_depth_topic, camera_info_topic;
  node->get_parameter(name_ + ".depth_topic", camera_depth_topic);
  node->get_parameter(name_ + ".info_topic", camera_info_topic);

  camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic,
    points_qos,
    std::bind(&DepthLayer::cameraInfoCallback, this, std::placeholders::_1));

  depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image,
    rclcpp_lifecycle::LifecycleNode>>(node, camera_depth_topic, rmw_qos_profile_sensor_data);
  depth_image_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::Image>>(
    *depth_image_sub_, *tf_, global_frame_, 10,
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    tf2::durationFromSec(transform_tolerance));
  depth_image_filter_->registerCallback(
    std::bind(&DepthLayer::depthImageCallback, this, std::placeholders::_1));
  observation_subscribers_.push_back(depth_image_sub_);
  observation_notifiers_.push_back(depth_image_filter_);
}

DepthLayer::~DepthLayer()
{
}

void DepthLayer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // Lock mutex before updating K
  std::lock_guard<std::mutex> lock(mutex_K_);

  float focal_pixels_ = msg->p[0];
  float center_x_ = msg->p[2];
  float center_y_ = msg->p[6];

  if (msg->binning_x == msg->binning_y)
  {
    if (msg->binning_x > 0)
    {
      K_ = (cv::Mat_<double>(3, 3) <<
        focal_pixels_/msg->binning_x, 0.0, center_x_/msg->binning_x,
        0.0, focal_pixels_/msg->binning_x, center_y_/msg->binning_x,
        0.0, 0.0, 1.0);
    }
    else
    {
      K_ = (cv::Mat_<double>(3, 3) <<
        focal_pixels_, 0.0, center_x_,
        0.0, focal_pixels_, center_y_,
        0.0, 0.0, 1.0);
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "binning_x is not equal to binning_y");
  }
}

void DepthLayer::depthImageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Lock mutex before using K
  std::lock_guard<std::mutex> lock(mutex_K_);

  if (K_.empty())
  {
    RCLCPP_WARN(LOGGER, "Camera info not yet received.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(LOGGER, "cv_bridge exception: %s", e.what());
    return;
  }

  // Clear with NANs?
  if (clear_nans_)
  {
    for (int i = 0; i < cv_ptr->image.rows * cv_ptr->image.cols; i++)
    {
      if (std::isnan(cv_ptr->image.at<float>(i)))
        cv_ptr->image.at<float>(i) = 25.0;
    }
  }

  // Filter noise
  cv::medianBlur(cv_ptr->image, cv_ptr->image, 5);

  // Convert to 3d
  cv::Mat points3d;
  depthTo3d(cv_ptr->image, K_, points3d);

  // Get normals
  if (normals_estimator_.empty())
  {
    normals_estimator_.reset(new RgbdNormals(cv_ptr->image.rows,
                                             cv_ptr->image.cols,
                                             cv_ptr->image.depth(),
                                             K_));
  }
  cv::Mat normals;
  (*normals_estimator_)(points3d, normals);

  // Find plane(s)
  if (plane_estimator_.empty())
  {
    plane_estimator_.reset(new RgbdPlane());
    // Model parameters are based on notes in opencv_candidate
    plane_estimator_->setSensorErrorA(0.0075);
    plane_estimator_->setSensorErrorB(0.0);
    plane_estimator_->setSensorErrorC(0.0);
    // Image/cloud height/width must be multiple of block size
    plane_estimator_->setBlockSize(40);
    // Distance a point can be from plane and still be part of it
    plane_estimator_->setThreshold(observations_threshold_);
    // Minimum cluster size to be a plane
    plane_estimator_->setMinSize(1000);
  }
  cv::Mat planes_mask;
  std::vector<cv::Vec4f> plane_coefficients;
  (*plane_estimator_)(points3d, normals, planes_mask, plane_coefficients);

  cv::Vec4f ground_plane;
  for (size_t i = 0; i < plane_coefficients.size(); i++)
  {
    // check plane orientation
    if ((fabs(0.0 - plane_coefficients[i][0]) <= ground_threshold_) &&
        (fabs(1.0 + plane_coefficients[i][1]) <= ground_threshold_) &&
        (fabs(0.0 - plane_coefficients[i][2]) <= ground_threshold_))
    {
      ground_plane = plane_coefficients[i];
      break;
    }
  }

  // Check that ground plane actually exists, so it doesn't count as marking observations
  if (ground_plane[0] == 0.0 && ground_plane[1] == 0.0 &&
      ground_plane[2] == 0.0 && ground_plane[3] == 0.0)
  {
    RCLCPP_WARN(LOGGER, "Invalid ground plane.");
    return;
  }

  cv::Mat channels[3];
  cv::split(points3d, channels);

  // Create clearing cloud
  int clearing_points = 0;
  sensor_msgs::msg::PointCloud2 clearing_msg;
  clearing_msg.header.stamp = msg->header.stamp;
  clearing_msg.header.frame_id = msg->header.frame_id;
  sensor_msgs::PointCloud2Modifier clearing_mod(clearing_msg);
  clearing_mod.setPointCloud2FieldsByString(1, "xyz");
  clearing_mod.resize(cv_ptr->image.rows * cv_ptr->image.cols);
  sensor_msgs::PointCloud2Iterator<float> clearing_x(clearing_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> clearing_y(clearing_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> clearing_z(clearing_msg, "z");

  // Create marking cloud
  int marking_points = 0;
  sensor_msgs::msg::PointCloud2 marking_msg;
  marking_msg.header.stamp = msg->header.stamp;
  marking_msg.header.frame_id = msg->header.frame_id;
  sensor_msgs::PointCloud2Modifier marking_mod(marking_msg);
  marking_mod.setPointCloud2FieldsByString(1, "xyz");
  marking_mod.resize(cv_ptr->image.rows * cv_ptr->image.cols);
  sensor_msgs::PointCloud2Iterator<float> marking_x(marking_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> marking_y(marking_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> marking_z(marking_msg, "z");

  // Put points in clearing/marking clouds
  for (size_t i = 0; i < points3d.rows; i++)
  {
    for (size_t j = 0; j < points3d.cols; j++)
    {
      // Get next point
      float x = channels[0].at<float>(i, j);
      float y = channels[1].at<float>(i, j);
      float z = channels[2].at<float>(i, j);
      // Check point validity
      if (x != 0.0 && !std::isnan(x) &&
          y != 0.0 && !std::isnan(y) &&
          z != 0.0 && !std::isnan(z))
      {
        // Check if point is part of the ground plane
        if (fabs(ground_plane[0] * x +
                 ground_plane[1] * y +
                 ground_plane[2] * z +
                 ground_plane[3]) <= observations_threshold_)
        {
          if (clear_with_skipped_rays_)
          {
            // If edge rays are to be used for clearing, go ahead and add them now.
            *clearing_x = x;
            *clearing_y = y;
            *clearing_z = z;
            ++clearing_points;
            ++clearing_x;
            ++clearing_y;
            ++clearing_z;
          }
          else if (i < skip_rays_top_ ||
                   i >= points3d.rows - skip_rays_bottom_ ||
                   j < skip_rays_left_ ||
                   j >= points3d.cols - skip_rays_right_)
          {
            // Do not consider boundary points for clearing since they are very noisy.
          }
          else 
          {
            // If edge rays are not to be used for clearing, only add them after the edge check.
            *clearing_x = x;
            *clearing_y = y;
            *clearing_z = z;
            ++clearing_points;
            ++clearing_x;
            ++clearing_y;
            ++clearing_z;
          }
        }
        else  // is marking
        {
          // Do not consider boundary points for obstacles marking since they are very noisy.
          if (i < skip_rays_top_ ||
              i >= points3d.rows - skip_rays_bottom_ ||
              j < skip_rays_left_ ||
              j >= points3d.cols - skip_rays_right_)
          {
            continue;
          }

          *marking_x = x;
          *marking_y = y;
          *marking_z = z;
          ++marking_points;
          ++marking_x;
          ++marking_y;
          ++marking_z;
        }
      }
    }  // for j (y)
  }  // for i (x)

  clearing_mod.resize(clearing_points);
  marking_mod.resize(marking_points);

  // Publish and buffer our clearing point cloud
  if (publish_observations_)
  {
    clearing_pub_->publish(clearing_msg);
    marking_pub_->publish(marking_msg);
  }

  // buffer the observations
  clearing_buf_->lock();
  clearing_buf_->bufferCloud(clearing_msg);
  clearing_buf_->unlock();

  marking_buf_->lock();
  marking_buf_->bufferCloud(marking_msg);
  marking_buf_->unlock();
}

}  // namespace nav2_costmap_2d

