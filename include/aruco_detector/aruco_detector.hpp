/*!*******************************************************************************************
 *  \file       aruco_detector.hpp
 *  \brief      Aruco detector header file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef ARUCO_DETECTOR_HPP_
#define ARUCO_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_core/tf_utils.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class ArucoDetector : public as2::Node
{
public:
  /**
   * @brief Construct a new Aruco Detector object
   */
  ArucoDetector();

  /**
   * @brief Destroy the Aruco Detector object
   */
  ~ArucoDetector(){};

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithID>::SharedPtr aruco_pose_pub_;
  std::shared_ptr<as2::sensors::Camera> aruco_img_transport_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::string camera_frame_;
  std::string ref_frame_;

  int n_aruco_ids_;
  int goal_id_ = 0;
  int n_first_samples_;
  int n_samples_ = 0;
  float aruco_size_;
  geometry_msgs::msg::PoseStamped position_;
  float max_distance_;
  std::string camera_model_;
  std::string distorsion_model_;
  bool camera_qos_reliable_;
  bool camera_params_available_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

  void setCameraInfo(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs);
  void loadParameters();
  void setCameraParameters(const sensor_msgs::msg::CameraInfo &_camera_info);

public:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);
  void camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info);
  bool filterPosition(const cv::Vec3d &aruco_position, float max_distance, int n_first_samples);
  cv::Vec3d convertPositionToRefFrame(const cv::Vec3d &_position, const std::string &_ref_frame);
};

#endif // ARUCO_DETECTOR_HPP_
