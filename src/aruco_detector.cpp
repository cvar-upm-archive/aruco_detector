/*!*******************************************************************************************
 *  \file       aruco_detector.cpp
 *  \brief      Aruco detector implementation file.
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

#include "aruco_detector.hpp"

ArucoDetector::ArucoDetector()
    : as2::Node("aruco_detector")
{
    std::string ns = this->get_namespace();
    loadParameters();

    aruco_pose_pub_ = this->create_publisher<as2_msgs::msg::PoseStampedWithID>(
        this->generate_local_name("aruco_pose"), rclcpp::SensorDataQoS());
    aruco_img_transport_ = std::make_shared<as2::sensors::Camera>(
        "aruco_img_topic", this);

    std::shared_ptr<const rclcpp::QoS> camera_qos;
    if (camera_qos_reliable_)
    {
        RCLCPP_INFO(get_logger(), "QoS Camera subscription: Reliable");
        camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
    }
    else
    {
        RCLCPP_INFO(get_logger(), "QoS Camera subscription: Sensor");
        camera_qos = std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());
    }

    cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        this->generate_global_name(as2_names::topics::sensor_measurements::camera + "/image_raw"),
        *camera_qos,
        std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        this->generate_global_name(as2_names::topics::sensor_measurements::camera + "/camera_info"),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&ArucoDetector::camerainfoCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
};

void ArucoDetector::loadParameters()
{
    std::string aruco_dictionary;
    std::string camera_frame;
    // this->declare_parameter("n_aruco_ids");
    this->declare_parameter<float>("aruco_size");
    this->declare_parameter<std::string>("camera_model");
    this->declare_parameter<std::string>("distortion_model");
    this->declare_parameter<bool>("camera_qos_reliable");
    this->declare_parameter<std::string>("ref_frame");
    this->declare_parameter<std::string>("camera_frame");
    this->declare_parameter<int>("goal_id");
    this->declare_parameter<float>("max_distance");
    this->declare_parameter<int>("n_first_samples");

    // this->get_parameter("n_aruco_ids", n_aruco_ids_);
    this->get_parameter("aruco_size", aruco_size_);
    this->get_parameter("camera_model", camera_model_);
    this->get_parameter("distortion_model", distorsion_model_);
    this->get_parameter("camera_qos_reliable", camera_qos_reliable_);
    this->get_parameter("ref_frame", ref_frame_);
    this->get_parameter("camera_frame", camera_frame);
    this->get_parameter("goal_id", goal_id_);
    this->get_parameter("max_distance", max_distance_);
    this->get_parameter("n_first_samples", n_first_samples_);

    std::string ns = this->get_namespace();
    camera_frame_ = as2::tf::generateTfName(ns, camera_frame);

    // RCLCPP_INFO(get_logger(), "Params: n_aruco_ids: %d", n_aruco_ids_);
    RCLCPP_INFO(get_logger(), "Params: aruco_size: %.3f m", aruco_size_);
    RCLCPP_INFO(get_logger(), "Params: goal_id: %d", goal_id_);
    RCLCPP_INFO(get_logger(), "Params: max_distance_: %.2f", max_distance_);
    RCLCPP_INFO(get_logger(), "Params: n_first_samples_: %d", n_first_samples_);
    RCLCPP_INFO(get_logger(), "Params: ref_frame_: %s", ref_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Params: camera_frame: %s", camera_frame_.c_str());
    

    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
}

void ArucoDetector::setCameraParameters(const sensor_msgs::msg::CameraInfo &_camera_info)
{
    camera_matrix_ = cv::Mat(3, 3, CV_64F);

    camera_matrix_.at<double>(0, 0) = _camera_info.k[0];
    camera_matrix_.at<double>(0, 1) = _camera_info.k[1];
    camera_matrix_.at<double>(0, 2) = _camera_info.k[2];
    camera_matrix_.at<double>(1, 0) = _camera_info.k[3];
    camera_matrix_.at<double>(1, 1) = _camera_info.k[4];
    camera_matrix_.at<double>(1, 2) = _camera_info.k[5];
    camera_matrix_.at<double>(2, 0) = _camera_info.k[6];
    camera_matrix_.at<double>(2, 1) = _camera_info.k[7];
    camera_matrix_.at<double>(2, 2) = _camera_info.k[8];

    std::cout << camera_matrix_ << std::endl;

    int n_discoeff = _camera_info.d.size();
    // RCLCPP_INFO(get_logger(), "number of D coeff: %i", n_discoeff);

    dist_coeffs_ = cv::Mat(1, n_discoeff, CV_64F);

    for (int i; i < n_discoeff; i++)
    {
        dist_coeffs_.at<double>(0, i) = _camera_info.d[i];
    }

    std::cout << dist_coeffs_ << std::endl;

    distorsion_model_ = _camera_info.distortion_model;

    if (camera_model_ == "fisheye")
    {
        RCLCPP_INFO(get_logger(), "Using FISHEYE camera model");
        if (n_discoeff != 4)
        {
            RCLCPP_ERROR(get_logger(), "FISHEYE distortion coefficients must be 4");
        }
    }
    if (camera_model_ == "pinhole")
        RCLCPP_INFO(get_logger(), "Using PINHOLE camera model");
    
    img_encoding_ = sensor_msgs::image_encodings::BGR8; 
    
    camera_params_available_ = true;
}

void ArucoDetector::setCameraInfo(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs)
{
    RCLCPP_DEBUG(get_logger(), "Setting camera info for aruco image");
    sensor_msgs::msg::CameraInfo camera_info;

    camera_info.k[0] = _camera_matrix.at<double>(0, 0);
    camera_info.k[1] = _camera_matrix.at<double>(0, 1);
    camera_info.k[2] = _camera_matrix.at<double>(0, 2);
    camera_info.k[3] = _camera_matrix.at<double>(1, 0);
    camera_info.k[4] = _camera_matrix.at<double>(1, 1);
    camera_info.k[5] = _camera_matrix.at<double>(1, 2);
    camera_info.k[6] = _camera_matrix.at<double>(2, 0);
    camera_info.k[7] = _camera_matrix.at<double>(2, 1);
    camera_info.k[8] = _camera_matrix.at<double>(2, 2);

    // FIXME
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 0));
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 1));
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 2));
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 3));
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 4));

    aruco_img_transport_->setParameters(camera_info, img_encoding_, camera_model_);
}

void ArucoDetector::camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
    RCLCPP_DEBUG(this->get_logger(), "Camera info received by callback");
    if (camera_params_available_)
    {
        return;
    }
    RCLCPP_INFO(get_logger(), "Setting camera info");
    setCameraParameters(*info);
}

bool ArucoDetector::filterPosition(const cv::Vec3d &aruco_position, float max_distance, int n_first_samples){
    
    const float alpha = 0.1;

    if (n_samples_ == 0){
        position_.pose.position.x = aruco_position[0];
        position_.pose.position.y = aruco_position[1];
        position_.pose.position.z = aruco_position[2];
        n_samples_++;
        return false;
    }
    //last_position_ = aruco_position;
    else {
        if (n_samples_ < n_first_samples){
            position_.pose.position.x = position_.pose.position.x * (1 - alpha) + aruco_position[0] * (alpha);
            position_.pose.position.y = position_.pose.position.y * (1 - alpha) + aruco_position[1] * (alpha);
            position_.pose.position.z = position_.pose.position.z * (1 - alpha) + aruco_position[2] * (alpha); 
            n_samples_ ++;
            return false;
        }
        else
        {
            Eigen::Vector3d new_position_v(aruco_position[0], aruco_position[1], aruco_position[2]);
            Eigen::Vector3d curr_position_v(position_.pose.position.x, position_.pose.position.y, position_.pose.position.z);

            float distance = (new_position_v - curr_position_v).norm();
            std::cout << "Last pose distance " << distance << std::endl;
            if (distance < max_distance){
                position_.pose.position.x = aruco_position[0];
                position_.pose.position.y = aruco_position[1];
                position_.pose.position.z = aruco_position[2];
                std::cout << "Sending pose " << distance << std::endl;

            }
            else
            {
                n_samples_ = 0;
                return false;
            }
        }
    }
    return true;
}

bool ArucoDetector::convertPositionToRefFrame(const cv::Vec3d &_position, cv::Vec3d &_new_position, const std::string &_ref_frame)
{

    try
    {
        auto pose_transform =
            tf_buffer_->lookupTransform(_ref_frame, camera_frame_, tf2::TimePointZero);

        geometry_msgs::msg::Vector3Stamped cam_pos_vs, ref_pos_vs;
        cam_pos_vs.vector.x = _position[0];
        cam_pos_vs.vector.y = _position[1];
        cam_pos_vs.vector.z = _position[2];

        tf2::doTransform(cam_pos_vs, ref_pos_vs, pose_transform);
        _new_position[0] = ref_pos_vs.vector.x;
        _new_position[1] = ref_pos_vs.vector.y;
        _new_position[2] = ref_pos_vs.vector.z;   
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Transform Failure: %s\n", ex.what()); // Print exception which was caught
        return false;
    }

    return true;
}

void ArucoDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
    RCLCPP_DEBUG(this->get_logger(), "Image received by callback");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, img_encoding_);
    }
    catch (cv_bridge::Exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "CV_bridge exception: %s\n", ex.what());
    }

    if (!camera_params_available_)
    {
        RCLCPP_WARN(get_logger(), "No camera parameters available");
        return;
    }

    // init ArUco detection
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();

    // detect markers on the fisheye, it's no worth it to detect over the rectified image
    cv::aruco::detectMarkers(cv_ptr->image, aruco_dict_, marker_corners, marker_ids, detector_params, rejected_candidates);
    if (marker_ids.empty()){
        if (n_samples_ > 0){
           n_samples_--; 
        } 
    }
    cv::Mat output_image = cv_ptr->image.clone();
    cv::Vec3d aruco_position, aruco_rotation;

    if (marker_ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids); // Draw markers to ensure orientation
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(marker_corners, aruco_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        for (int i = 0; i < rvecs.size(); ++i)
        {
            int id = marker_ids[i];

            if (id == goal_id_)
            {
                RCLCPP_INFO(this->get_logger(), "Marker %d detected", id);
                cv::Vec3d rvec = rvecs[i];
                cv::Vec3d tvec = tvecs[i];
                // cv::Vec3d t_gate;
                cv::Vec3d rout, tout;
                // t_gate[0] = -gate_size_ / 2.0f;
                cv::composeRT(rvec * 0, tvec * 0, rvec, tvec, rout, tout);
                aruco_position = tout;
                aruco_rotation = rout;
                cv::aruco::drawAxis(output_image, camera_matrix_, dist_coeffs_, rout, tout, 0.08625);
            }
        }
    }

    cv::Mat undistort_camera_matrix;
    cv::Mat rectified_image, cropped_image;
    cv::Rect roi;
    float alpha = 0.0;

    if (camera_model_ == "pinhole")
    {
        // use opencv functions to rectify output image using camera matrix and distortion coefficients
        // OPTION 1:
        RCLCPP_INFO_ONCE(get_logger(), "Undistort image with pinhole model");
        // undistort_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix_, dist_coeffs_, output_image.size(), alpha, output_image.size(), &roi);
        cv::undistort(output_image, rectified_image, camera_matrix_, dist_coeffs_);
        // cropped_image = rectified_image(roi);
    }

    if (camera_model_ == "fisheye")
    {
        RCLCPP_INFO_ONCE(get_logger(), "Undistort image with fisheye model");
        // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix_, dist_coeffs_, output_image.size(), cv::Matx33d::eye(), undistort_camera_matrix, alpha, output_image.size());
        cv::fisheye::undistortImage(output_image, rectified_image, camera_matrix_, dist_coeffs_);
        // auto marker0 = marker_corners[0];
        // cv::fisheye::undistortPoints(marker_corners[0], marker0, camera_matrix_, dist_coeffs_);
    }

    // cropped_image = rectified_image(roi);

    // OPTION 2:
    //  cv::Mat cropped_image;
    //  cv::OutputArray map1 = cv::noArray();
    //  cv::OutputArray map2 = cv::noArray();
    //  cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, output_image.size(),CV_32FC1, map1, map2);
    //  cv::remap(output_image, cropped_image, map1, map2, cv::INTER_LINEAR);

    sensor_msgs::msg::Image output_image_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified_image).toImageMsg().get());
    aruco_img_transport_->updateData(output_image_msg);

    // Publish path

    // if (marker_ids.size() > 0)
    // {
        //nav_msgs::msg::Path path;
        //as2_msgs::msg::TrajectoryWaypointsWithID path;
        //path.header.frame_id = "camera_link";
        
    for (int i = 0; i < marker_ids.size(); i++)
    {
        int id = marker_ids[i];
        if (id == goal_id_){  
            std::cout << aruco_position << std::endl;  
            cv::Vec3d aruco_global_position;     
            if (!convertPositionToRefFrame(aruco_position, aruco_global_position, ref_frame_))
            {
                break;
            }
            std::cout << aruco_global_position << std::endl;
            if (filterPosition(aruco_global_position, max_distance_, n_first_samples_))
            {
                as2_msgs::msg::PoseStampedWithID pose;
                pose.id = std::to_string(id);
                pose.header.frame_id = camera_frame_;
                pose.pose.position.x = aruco_global_position[0];
                pose.pose.position.y = aruco_global_position[1];
                pose.pose.position.z = aruco_global_position[2];
                tf2::Quaternion rot;
                rot.setRPY(aruco_rotation[2], aruco_rotation[0], aruco_rotation[1]); // 2, 0, 1 ?? 
                rot = rot.normalize();
                pose.pose.orientation.x = (double)rot.x();
                pose.pose.orientation.y = (double)rot.y();
                pose.pose.orientation.z = (double)rot.z();
                pose.pose.orientation.w = (double)rot.w();

                aruco_pose_pub_->publish(pose);
            }
        }
               //path.poses.emplace_back(pose); 
        //}    
    }
};
