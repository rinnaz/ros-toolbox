#pragma once

#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <rnrt_msgs/Markers.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MarkerDetector
{
public:
  MarkerDetector();
  ~MarkerDetector();
  void callback(const sensor_msgs::Image::ConstPtr& img) const;

private:
  void initParameters();
  void initArucoDictSelector();
  void readCameraParams();
  void readDetectorParams();

  geometry_msgs::Pose makeMarkerPose(const int& marker_id, const cv::Vec3d& rvec, const cv::Vec3d& tvec) const;

  geometry_msgs::Quaternion rotMatToQuat(const cv::Mat& rot_mat) const;
  geometry_msgs::TransformStamped makeTransformMsg(geometry_msgs::Pose& pose, const int& marker_id) const;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub_markers, m_pub_image;
  ros::Subscriber m_sub;

  const std::string m_node_name;
  std::string m_package_path;
  std::string m_camera_parameters_file;
  std::string m_detector_parameters_file;
  std::string m_source_camera_topic;
  std::string m_output_image_topic;
  std::string m_marker_pose_topic;
  std::string m_tf_parent_frame;
  std::string m_tf_child_frame_prefix;
  std::string m_aruco_dict_type;
  double m_marker_size;
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> m_aruco_dict_selector;

  cv::Ptr<cv::aruco::DetectorParameters> m_detector_parameters;
  cv::Mat m_camera_matrix;
  cv::Mat m_dist_coeffs;

  cv::Ptr<cv::aruco::Dictionary> m_dict;

  mutable std::vector<int> m_marker_ids;
  mutable std::vector<std::vector<cv::Point2f>> m_marker_corners;
  mutable std::vector<cv::Vec3d> m_rvecs, m_tvecs;
};
