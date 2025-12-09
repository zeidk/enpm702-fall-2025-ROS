/**
 * @file aruco_detector_node.cpp
 * @author zeidk (zeidk@umd.edu)
 * @brief ArUco marker detector node with KDL transform examples
 *
 * This node demonstrates two approaches for coordinate transforms:
 *
 * 1. TF2 Broadcaster/Listener (standard ROS approach)
 *    - Broadcasts: camera_frame -> aruco_marker_X
 *    - Listens: odom -> camera_frame
 *    - Uses tf2::doTransform() for pose transformation
 *
 * 2. KDL (Kinematics and Dynamics Library)
 *    - Manual transform composition using KDL::Frame
 *    - Explicit matrix math: T_odom_marker = T_odom_camera * T_camera_marker
 *    - Educational: shows what TF does "under the hood"
 *
 * Publications:
 *   - ~/detected_markers (PoseArray): Marker poses in camera frame
 *   - ~/detected_markers_odom (PoseArray): Marker poses in odom frame
 *   - ~/marker_pose (PoseStamped): First marker pose in odom frame
 *   - ~/debug_image (Image): Annotated debug image
 *
 * @version 0.1
 * @date 2025-12-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "aruco_tf_demo/aruco_detector_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace aruco_tf_demo {

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions& options)
    : Node("aruco_detector", options) {
  // ===========================================================================
  // Declare and get parameters
  // ===========================================================================
  this->declare_parameter("marker_size", 0.15);
  this->declare_parameter("camera_frame", "oak_rgb_camera_optical_frame");
  this->declare_parameter("marker_frame_prefix", "aruco_marker_");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("publish_debug_image", true);
  this->declare_parameter("publish_odom_poses", true);
  this->declare_parameter("use_kdl", false);  // Use KDL instead of TF2 listener
  this->declare_parameter("camera_topic", "/oak/rgb/color");
  this->declare_parameter("camera_info_topic", "/oak/rgb/camera_info");
  this->declare_parameter("dictionary_id", 0);  // DICT_4X4_50 = 0

  marker_size_ = this->get_parameter("marker_size").as_double();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  marker_frame_prefix_ = this->get_parameter("marker_frame_prefix").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
  publish_odom_poses_ = this->get_parameter("publish_odom_poses").as_bool();
  use_kdl_ = this->get_parameter("use_kdl").as_bool();

  auto camera_topic = this->get_parameter("camera_topic").as_string();
  auto camera_info_topic = this->get_parameter("camera_info_topic").as_string();
  auto dictionary_id = this->get_parameter("dictionary_id").as_int();

  // ===========================================================================
  // Initialize ArUco detector
  // ===========================================================================
  aruco_dict_ = cv::aruco::getPredefinedDictionary(dictionary_id);
  detector_params_ = cv::aruco::DetectorParameters::create();

  // Tune detection parameters for better performance
  detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params_->cornerRefinementWinSize = 5;
  detector_params_->cornerRefinementMaxIterations = 30;
  detector_params_->cornerRefinementMinAccuracy = 0.1;

  // ===========================================================================
  // Initialize TF broadcaster
  // ===========================================================================
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ===========================================================================
  // Initialize TF listener and buffer
  // ===========================================================================
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===========================================================================
  // Initialize subscribers
  // ===========================================================================
  // Use RELIABLE QoS to match ROSbot camera driver
  rmw_qos_profile_t camera_qos_profile = rmw_qos_profile_default;
  camera_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  camera_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  // Using image_transport for efficient image transmission
  image_sub_ = image_transport::create_subscription(
      this, camera_topic,
      std::bind(&ArucoDetectorNode::image_callback, this,
                std::placeholders::_1),
      "raw", camera_qos_profile);

  // Use RELIABLE QoS to match ROSbot camera driver
  auto camera_qos = rclcpp::QoS(10)
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, camera_qos,
      std::bind(&ArucoDetectorNode::camera_info_callback, this,
                std::placeholders::_1));

  // ===========================================================================
  // Initialize publishers
  // ===========================================================================
  pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "~/detected_markers", 10);

  pose_array_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "~/detected_markers_odom", 10);

  marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/marker_pose", 10);

  if (publish_debug_image_) {
    debug_image_pub_ = image_transport::create_publisher(this, "~/debug_image");
  }

  // ===========================================================================
  // Log initialization
  // ===========================================================================
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "ArUco Detector Node initialized");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "  Marker size: %.3f m", marker_size_);
  RCLCPP_INFO(this->get_logger(), "  Camera frame: %s", camera_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Odom frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Marker frame prefix: %s",
              marker_frame_prefix_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Dictionary ID: %ld", dictionary_id);
  RCLCPP_INFO(this->get_logger(), "  Publish TF: %s",
              publish_tf_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Publish debug image: %s",
              publish_debug_image_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Publish odom poses: %s",
              publish_odom_poses_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Use KDL for transforms: %s",
              use_kdl_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Camera topic: %s", camera_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Camera info topic: %s",
              camera_info_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "========================================");
  if (use_kdl_) {
    RCLCPP_INFO(this->get_logger(),
                "Using KDL for manual transform composition");
    RCLCPP_INFO(this->get_logger(), "  Transform chain: %s -> %s -> marker",
                odom_frame_.c_str(), camera_frame_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Using TF2 listener for transform lookups");
  }
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Waiting for camera info...");
}

void ArucoDetectorNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  if (camera_info_received_) {
    return;  // Only process once
  }

  // Validate camera info
  if (msg->k[0] == 0.0 || msg->k[4] == 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid camera info received (zero focal length)");
    return;
  }

  // Extract camera intrinsic matrix (3x3)
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
  }

  // Extract distortion coefficients
  if (msg->d.empty()) {
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    RCLCPP_WARN(
        this->get_logger(),
        "No distortion coefficients provided, assuming zero distortion");
  } else {
    dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i{0}; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
  }

  camera_info_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Camera info received!");
  RCLCPP_INFO(this->get_logger(), "  Image size: %dx%d", msg->width,
              msg->height);
  RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
              camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
              camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
}

void ArucoDetectorNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  // Wait for camera calibration
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for camera info...");
    return;
  }

  // Convert ROS image to OpenCV
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding == sensor_msgs::image_encodings::BGR8 ||
        msg->encoding == sensor_msgs::image_encodings::RGB8 ||
        msg->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvShare(msg);
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to grayscale for detection if needed
  cv::Mat gray;
  if (cv_ptr->image.channels() == 3) {
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
    } else {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    }
  } else {
    gray = cv_ptr->image;
  }

  // Detect markers
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> rejected;
  detect_markers(gray, corners, ids, rejected);

  // If no markers detected, optionally publish debug image
  if (ids.empty()) {
    if (publish_debug_image_) {
      cv::Mat debug_image;
      if (cv_ptr->image.channels() == 1) {
        cv::cvtColor(cv_ptr->image, debug_image, cv::COLOR_GRAY2BGR);
      } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(cv_ptr->image, debug_image, cv::COLOR_RGB2BGR);
      } else {
        debug_image = cv_ptr->image.clone();
      }
      cv::putText(debug_image, "No markers detected", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

      auto debug_msg =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8,
                             debug_image)
              .toImageMsg();
      debug_image_pub_.publish(debug_msg);
    }
    return;
  }

  // Estimate poses
  std::vector<cv::Vec3d> rvecs, tvecs;
  estimate_poses(corners, ids, rvecs, tvecs);

  // Publish transforms (camera_frame -> aruco_marker_X)
  if (publish_tf_) {
    publish_transforms(ids, rvecs, tvecs, msg->header.stamp);
  }

  // Publish pose array in camera frame
  publish_pose_array(ids, rvecs, tvecs, msg->header);

  // Publish poses in odom frame (using TF2 listener or KDL)
  if (publish_odom_poses_) {
    if (use_kdl_) {
      // Use KDL for manual transform composition
      compute_poses_with_kdl(ids, rvecs, tvecs, msg->header.stamp);
    } else {
      // Use TF2 listener (standard ROS approach)
      publish_poses_in_odom_frame(ids, rvecs, tvecs, msg->header.stamp);
    }
  }

  // Publish debug image
  if (publish_debug_image_) {
    cv::Mat debug_image;
    if (cv_ptr->image.channels() == 1) {
      cv::cvtColor(cv_ptr->image, debug_image, cv::COLOR_GRAY2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(cv_ptr->image, debug_image, cv::COLOR_RGB2BGR);
    } else {
      debug_image = cv_ptr->image.clone();
    }
    publish_debug_image(debug_image, corners, ids, rvecs, tvecs, msg->header);
  }

  // Log detection info (throttled)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Detected %zu marker(s): %s", ids.size(),
                       format_ids(ids).c_str());
}

void ArucoDetectorNode::detect_markers(
    const cv::Mat& image, std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& rejected) {
  cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, detector_params_,
                           rejected);

  RCLCPP_DEBUG(this->get_logger(),
               "Detection: %zu markers found, %zu candidates rejected",
               ids.size(), rejected.size());
}

void ArucoDetectorNode::estimate_poses(
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids, std::vector<cv::Vec3d>& rvecs,
    std::vector<cv::Vec3d>& tvecs) {
  if (corners.empty()) {
    return;
  }

  cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_,
                                       dist_coeffs_, rvecs, tvecs);

  for (size_t i{0}; i < ids.size(); ++i) {
    double distance = cv::norm(tvecs[i]);
    RCLCPP_DEBUG(this->get_logger(),
                 "Marker %d: position=(%.3f, %.3f, %.3f) distance=%.3f m",
                 ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2], distance);
  }
}

void ArucoDetectorNode::publish_transforms(const std::vector<int>& ids,
                                           const std::vector<cv::Vec3d>& rvecs,
                                           const std::vector<cv::Vec3d>& tvecs,
                                           const rclcpp::Time& stamp) {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(ids.size());

  for (size_t i{0}; i < ids.size(); ++i) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = camera_frame_;
    transform.child_frame_id = marker_frame_prefix_ + std::to_string(ids[i]);

    // Set translation
    transform.transform.translation.x = tvecs[i][0];
    transform.transform.translation.y = tvecs[i][1];
    transform.transform.translation.z = tvecs[i][2];

    // Convert rotation vector to quaternion
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs[i], rotation_matrix);

    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));

    tf2::Quaternion quaternion;
    tf_rotation.getRotation(quaternion);
    quaternion.normalize();

    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    transforms.push_back(transform);
  }

  // Broadcast all transforms at once
  tf_broadcaster_->sendTransform(transforms);
}

void ArucoDetectorNode::publish_pose_array(
    const std::vector<int>& ids, const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs, const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = header;
  pose_array.header.frame_id = camera_frame_;

  for (size_t i{0}; i < ids.size(); ++i) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = tvecs[i][0];
    pose.position.y = tvecs[i][1];
    pose.position.z = tvecs[i][2];

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs[i], rotation_matrix);

    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));

    tf2::Quaternion quaternion;
    tf_rotation.getRotation(quaternion);
    quaternion.normalize();

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    pose_array.poses.push_back(pose);
  }

  pose_array_pub_->publish(pose_array);
}

bool ArucoDetectorNode::lookup_transform(
    const std::string& target_frame, const std::string& source_frame,
    const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& transform) {
  try {
    // Use a timeout for the lookup
    transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.1));
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped ArucoDetectorNode::rvec_tvec_to_pose(
    const cv::Vec3d& rvec, const cv::Vec3d& tvec,
    const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  pose.pose.position.x = tvec[0];
  pose.pose.position.y = tvec[1];
  pose.pose.position.z = tvec[2];

  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);

  tf2::Matrix3x3 tf_rotation(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
      rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
      rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
      rotation_matrix.at<double>(2, 2));

  tf2::Quaternion quaternion;
  tf_rotation.getRotation(quaternion);
  quaternion.normalize();

  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();

  return pose;
}

void ArucoDetectorNode::publish_poses_in_odom_frame(
    const std::vector<int>& ids, const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs, const rclcpp::Time& stamp) {
  // Look up transform from odom to camera frame
  geometry_msgs::msg::TransformStamped odom_to_camera;
  if (!lookup_transform(odom_frame_, camera_frame_, stamp, odom_to_camera)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Cannot look up transform from %s to %s",
                         odom_frame_.c_str(), camera_frame_.c_str());
    return;
  }

  geometry_msgs::msg::PoseArray pose_array_odom;
  pose_array_odom.header.stamp = stamp;
  pose_array_odom.header.frame_id = odom_frame_;

  for (size_t i{0}; i < ids.size(); ++i) {
    // Create pose in camera frame
    std_msgs::msg::Header camera_header;
    camera_header.stamp = stamp;
    camera_header.frame_id = camera_frame_;

    geometry_msgs::msg::PoseStamped pose_camera =
        rvec_tvec_to_pose(rvecs[i], tvecs[i], camera_header);

    // Transform pose from camera frame to odom frame
    geometry_msgs::msg::PoseStamped pose_odom;
    try {
      tf2::doTransform(pose_camera, pose_odom, odom_to_camera);
      pose_odom.header.frame_id = odom_frame_;
      pose_array_odom.poses.push_back(pose_odom.pose);

      // Publish single marker pose (first detected marker)
      if (i == 0) {
        marker_pose_pub_->publish(pose_odom);
      }

      RCLCPP_DEBUG(this->get_logger(),
                   "Marker %d in %s: position=(%.3f, %.3f, %.3f)", ids[i],
                   odom_frame_.c_str(), pose_odom.pose.position.x,
                   pose_odom.pose.position.y, pose_odom.pose.position.z);

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
  }

  if (!pose_array_odom.poses.empty()) {
    pose_array_odom_pub_->publish(pose_array_odom);
  }
}

void ArucoDetectorNode::publish_debug_image(
    const cv::Mat& image, const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids, const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs, const std_msgs::msg::Header& header) {
  cv::Mat debug_image = image.clone();

  // Draw detected marker boundaries and IDs
  cv::aruco::drawDetectedMarkers(debug_image, corners, ids);

  // Draw coordinate axes for each marker
  for (size_t i{0}; i < ids.size(); ++i) {
    cv::drawFrameAxes(debug_image, camera_matrix_, dist_coeffs_, rvecs[i],
                      tvecs[i], static_cast<float>(marker_size_ * 0.5));

    // Draw distance text
    double distance = cv::norm(tvecs[i]);
    std::stringstream ss;
    ss << "ID:" << ids[i] << " D:" << std::fixed << std::setprecision(2)
       << distance << "m";
    std::string dist_text = ss.str();

    cv::Point2f center(0, 0);
    for (const auto& corner : corners[i]) {
      center += corner;
    }
    center *= 0.25f;

    cv::putText(debug_image, dist_text,
                cv::Point(static_cast<int>(center.x) - 50,
                          static_cast<int>(center.y) - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
  }

  // Add header info
  std::string info_text = "Markers detected: " + std::to_string(ids.size());
  cv::putText(debug_image, info_text, cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

  // Convert and publish
  auto debug_msg = cv_bridge::CvImage(
                       header, sensor_msgs::image_encodings::BGR8, debug_image)
                       .toImageMsg();
  debug_image_pub_.publish(debug_msg);
}

std::string ArucoDetectorNode::format_ids(const std::vector<int>& ids) {
  std::string result = "[";
  for (size_t i{0}; i < ids.size(); ++i) {
    result += std::to_string(ids[i]);
    if (i < ids.size() - 1) {
      result += ", ";
    }
  }
  result += "]";
  return result;
}

// =============================================================================
// KDL Transform Methods
// =============================================================================
// These methods demonstrate manual coordinate transform composition using KDL.
// This is an alternative to using TF2's lookupTransform and doTransform.
//
// Key KDL concepts:
//   - KDL::Frame: A 4x4 homogeneous transformation matrix (rotation +
//   translation)
//   - KDL::Rotation: 3x3 rotation matrix
//   - KDL::Vector: 3D vector (x, y, z)
//   - Frame composition: T_A_C = T_A_B * T_B_C (transforms compose
//   left-to-right)
// =============================================================================

KDL::Frame ArucoDetectorNode::rvec_tvec_to_kdl_frame(const cv::Vec3d& rvec,
                                                     const cv::Vec3d& tvec) {
  // Convert OpenCV rotation vector (Rodrigues) to rotation matrix
  cv::Mat rotation_matrix_cv;
  cv::Rodrigues(rvec, rotation_matrix_cv);

  // Create KDL rotation from OpenCV rotation matrix
  // KDL::Rotation constructor takes elements in row-major order
  KDL::Rotation rotation(
      rotation_matrix_cv.at<double>(0, 0), rotation_matrix_cv.at<double>(0, 1),
      rotation_matrix_cv.at<double>(0, 2), rotation_matrix_cv.at<double>(1, 0),
      rotation_matrix_cv.at<double>(1, 1), rotation_matrix_cv.at<double>(1, 2),
      rotation_matrix_cv.at<double>(2, 0), rotation_matrix_cv.at<double>(2, 1),
      rotation_matrix_cv.at<double>(2, 2));

  // Create KDL vector from translation
  KDL::Vector translation(tvec[0], tvec[1], tvec[2]);

  // Return KDL frame (rotation + translation)
  return KDL::Frame(rotation, translation);
}

geometry_msgs::msg::Pose ArucoDetectorNode::kdl_frame_to_pose(
    const KDL::Frame& frame) {
  geometry_msgs::msg::Pose pose;

  // Extract translation
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();

  // Extract rotation as quaternion
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  pose.orientation.x = x;
  pose.orientation.y = y;
  pose.orientation.z = z;
  pose.orientation.w = w;

  return pose;
}

KDL::Frame ArucoDetectorNode::transform_to_kdl_frame(
    const geometry_msgs::msg::Transform& transform) {
  // Create KDL vector from translation
  KDL::Vector translation(transform.translation.x, transform.translation.y,
                          transform.translation.z);

  // Create KDL rotation from quaternion
  KDL::Rotation rotation =
      KDL::Rotation::Quaternion(transform.rotation.x, transform.rotation.y,
                                transform.rotation.z, transform.rotation.w);

  return KDL::Frame(rotation, translation);
}

void ArucoDetectorNode::log_kdl_frame(const std::string& frame_name,
                                      const KDL::Frame& frame) {
  double roll, pitch, yaw;
  frame.M.GetRPY(roll, pitch, yaw);

  double x{};
  double y{};
  double z{};
  double w{};

  frame.M.GetQuaternion(x, y, z, w);

  RCLCPP_INFO(
      this->get_logger(),
      "KDL Frame [%s]: pos=(%.3f, %.3f, %.3f) rot=(%.3f, %.3f, %.3f, %.3f)",
      frame_name.c_str(), frame.p.x(), frame.p.y(), frame.p.z(), x, y, z, w);
}

void ArucoDetectorNode::compute_poses_with_kdl(
    const std::vector<int>& ids, const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs, const rclcpp::Time& stamp) {
  // ===========================================================================
  // KDL Transform Composition Example
  // ===========================================================================
  //
  // Goal: Compute marker pose in odom frame
  //
  // We have:
  //   - T_camera_marker: Marker pose in camera frame (from ArUco detection)
  //   - T_odom_camera: Camera pose in odom frame (from TF tree)
  //
  // We want:
  //   - T_odom_marker: Marker pose in odom frame
  //
  // Transform composition formula:
  //   T_odom_marker = T_odom_camera * T_camera_marker
  //
  // This is equivalent to what TF2's doTransform() does internally.
  // ===========================================================================

  // Step 1: Look up transform from odom to camera frame
  // (We still use TF2 to get this transform, but then do the math with KDL)
  geometry_msgs::msg::TransformStamped odom_to_camera_tf;
  if (!lookup_transform(odom_frame_, camera_frame_, stamp, odom_to_camera_tf)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[KDL] Cannot look up transform from %s to %s",
                         odom_frame_.c_str(), camera_frame_.c_str());
    return;
  }

  // Step 2: Convert TF transform to KDL frame
  KDL::Frame T_odom_camera =
      transform_to_kdl_frame(odom_to_camera_tf.transform);

  // Log the odom->camera transform (throttled)
  static int log_counter{0};
  if (++log_counter % 30 == 0) {  // Log every ~1 second at 30Hz
    log_kdl_frame("T_odom_camera", T_odom_camera);
  }

  // Prepare output messages
  geometry_msgs::msg::PoseArray pose_array_odom;
  pose_array_odom.header.stamp = stamp;
  pose_array_odom.header.frame_id = odom_frame_;

  // Step 3: For each detected marker, compute pose in odom frame
  for (size_t i{0}; i < ids.size(); ++i) {
    // Convert ArUco detection (rvec, tvec) to KDL frame
    // This is T_camera_marker: marker pose expressed in camera frame
    KDL::Frame T_camera_marker = rvec_tvec_to_kdl_frame(rvecs[i], tvecs[i]);

    // ===========================================================================
    // KEY STEP: Transform composition using KDL
    // ===========================================================================
    // T_odom_marker = T_odom_camera * T_camera_marker
    //
    // In KDL, the * operator composes frames:
    //   - Left frame transforms points FROM its frame TO the parent frame
    //   - Result: marker position expressed in odom frame
    // ===========================================================================
    KDL::Frame T_odom_marker = T_odom_camera * T_camera_marker;

    // Log individual marker transforms (throttled)
    if (log_counter % 30 == 0) {
      std::string marker_name = "T_camera_marker_" + std::to_string(ids[i]);
      log_kdl_frame(marker_name, T_camera_marker);

      std::string result_name = "T_odom_marker_" + std::to_string(ids[i]);
      log_kdl_frame(result_name, T_odom_marker);
    }

    // Convert KDL frame to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose_odom = kdl_frame_to_pose(T_odom_marker);
    pose_array_odom.poses.push_back(pose_odom);

    // Publish single marker pose (first detected marker)
    if (i == 0) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = stamp;
      pose_stamped.header.frame_id = odom_frame_;
      pose_stamped.pose = pose_odom;
      marker_pose_pub_->publish(pose_stamped);
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "[KDL] Marker %d in %s: position=(%.3f, %.3f, %.3f)", ids[i],
                 odom_frame_.c_str(), pose_odom.position.x,
                 pose_odom.position.y, pose_odom.position.z);
  }

  // Publish pose array
  if (!pose_array_odom.poses.empty()) {
    pose_array_odom_pub_->publish(pose_array_odom);
  }
}

}  // namespace aruco_tf_demo

// =============================================================================
// Main function
// =============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<aruco_tf_demo::ArucoDetectorNode>();

  RCLCPP_INFO(node->get_logger(),
              "ArUco Detector Node started. Press Ctrl+C to exit.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}