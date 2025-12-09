/**
 * @file aruco_detector_node.hpp
 * @author zeidk (zeidk@umd.edu)
 * @brief ArUco marker detector node for ROS 2 with KDL transform examples
 *
 * This node detects ArUco markers in camera images and publishes
 * their poses as TF transforms. It demonstrates two approaches:
 *
 * 1. TF2 Broadcaster/Listener (standard ROS approach)
 * 2. KDL (Kinematics and Dynamics Library) for manual transforms
 *
 * KDL provides explicit control over coordinate frame math, useful for:
 *   - Understanding the math behind TF transforms
 *   - Custom transform chains not in the TF tree
 *   - Kinematics computations in manipulation
 * @version 0.1
 * @date 2025-12-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#pragma once

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

// TF2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// KDL includes for manual coordinate transforms
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace aruco_tf_demo
{

/**
 * @class ArucoDetectorNode
 * @brief ROS 2 node for ArUco marker detection and TF publishing
 *
 * This node subscribes to camera images and camera info, detects ArUco
 * markers, estimates their 6-DOF poses, and broadcasts TF transforms.
 * It also includes a TF listener to look up marker poses in other frames
 * (e.g., odom frame).
 */
class ArucoDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit ArucoDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ArucoDetectorNode() = default;

private:
  /**
   * @brief Callback for camera image messages
   * @param msg Image message
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief Callback for camera info messages
   * @param msg Camera info message
   */
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  /**
   * @brief Detect ArUco markers in an image
   * @param image Input image (grayscale)
   * @param corners Output vector of marker corners
   * @param ids Output vector of marker IDs
   * @param rejected Output vector of rejected candidate corners
   */
  void detect_markers(
    const cv::Mat & image,
    std::vector<std::vector<cv::Point2f>> & corners,
    std::vector<int> & ids,
    std::vector<std::vector<cv::Point2f>> & rejected);

  /**
   * @brief Estimate pose of detected markers
   * @param corners Marker corners
   * @param ids Marker IDs
   * @param rvecs Output rotation vectors (Rodrigues format)
   * @param tvecs Output translation vectors
   */
  void estimate_poses(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    std::vector<cv::Vec3d> & rvecs,
    std::vector<cv::Vec3d> & tvecs);

  /**
   * @brief Publish TF transforms for detected markers
   * @param ids Marker IDs
   * @param rvecs Rotation vectors
   * @param tvecs Translation vectors
   * @param stamp Timestamp for transforms
   */
  void publish_transforms(
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const rclcpp::Time & stamp);

  /**
   * @brief Publish pose array message (in camera frame)
   * @param ids Marker IDs
   * @param rvecs Rotation vectors
   * @param tvecs Translation vectors
   * @param header Message header
   */
  void publish_pose_array(
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const std_msgs::msg::Header & header);

  /**
   * @brief Publish marker poses in the odom frame
   * @param ids Marker IDs
   * @param rvecs Rotation vectors
   * @param tvecs Translation vectors
   * @param stamp Timestamp
   */
  void publish_poses_in_odom_frame(
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const rclcpp::Time & stamp);

  /**
   * @brief Look up transform between two frames
   * @param target_frame Target frame
   * @param source_frame Source frame
   * @param time Timestamp for lookup
   * @param transform Output transform
   * @return True if transform was found
   */
  bool lookup_transform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & time,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief Convert rotation vector and translation vector to PoseStamped
   * @param rvec Rotation vector (Rodrigues)
   * @param tvec Translation vector
   * @param header Header for the pose
   * @return PoseStamped message
   */
  geometry_msgs::msg::PoseStamped rvec_tvec_to_pose(
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec,
    const std_msgs::msg::Header & header);

  /**
   * @brief Publish debug image with annotations
   * @param image Original image
   * @param corners Detected marker corners
   * @param ids Marker IDs
   * @param rvecs Rotation vectors
   * @param tvecs Translation vectors
   * @param header Message header
   */
  void publish_debug_image(
    const cv::Mat & image,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const std_msgs::msg::Header & header);

  /**
   * @brief Format marker IDs as string for logging
   * @param ids Vector of marker IDs
   * @return Formatted string
   */
  std::string format_ids(const std::vector<int> & ids);

  // ===========================================================================
  // KDL Transform Methods (Alternative to TF2 Listener)
  // ===========================================================================

  /**
   * @brief Convert OpenCV rotation vector and translation vector to KDL Frame
   * @param rvec Rotation vector (Rodrigues format)
   * @param tvec Translation vector
   * @return KDL::Frame representing the transform
   */
  KDL::Frame rvec_tvec_to_kdl_frame(const cv::Vec3d & rvec, const cv::Vec3d & tvec);

  /**
   * @brief Convert KDL Frame to geometry_msgs::Pose
   * @param frame KDL frame
   * @return Pose message
   */
  geometry_msgs::msg::Pose kdl_frame_to_pose(const KDL::Frame & frame);

  /**
   * @brief Convert geometry_msgs::Transform to KDL Frame
   * @param transform Transform message
   * @return KDL::Frame
   */
  KDL::Frame transform_to_kdl_frame(const geometry_msgs::msg::Transform & transform);

  /**
   * @brief Compute marker poses in odom frame using KDL (manual transform chain)
   * 
   * This demonstrates how to manually compose transforms using KDL,
   * as an alternative to using TF2's lookupTransform and doTransform.
   * 
   * Transform chain: odom -> base_link -> camera_frame -> marker
   * Result: marker pose expressed in odom frame
   * 
   * @param ids Marker IDs
   * @param rvecs Rotation vectors (marker poses in camera frame)
   * @param tvecs Translation vectors
   * @param stamp Timestamp for TF lookups
   */
  void compute_poses_with_kdl(
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const rclcpp::Time & stamp);

  /**
   * @brief Log KDL frame information for debugging
   * @param frame_name Name of the frame for logging
   * @param frame KDL frame to log
   */
  void log_kdl_frame(const std::string & frame_name, const KDL::Frame & frame);

  // Subscribers
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
  image_transport::Publisher debug_image_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF listener and buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ArUco detection
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // Camera intrinsics
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_{false};

  // Parameters
  double marker_size_;
  std::string camera_frame_;
  std::string marker_frame_prefix_;
  std::string odom_frame_;
  bool publish_tf_;
  bool publish_debug_image_;
  bool publish_odom_poses_;
  bool use_kdl_;  // Use KDL for transforms instead of TF2 listener
};

}  // namespace aruco_tf_demo