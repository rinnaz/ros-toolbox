#include "rnrt_aruco_detector/rnrt_aruco_detector.h"

MarkerDetector::MarkerDetector() : m_node_name{ ros::this_node::getName() }
{
  // Get node parameters set by launch file
  initParameters();
  initArucoDictSelector();

  // Init aruco dictionary
  try
  {
    m_dict = cv::aruco::getPredefinedDictionary(m_aruco_dict_selector.at(m_aruco_dict_type));
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Error occured: %s ", e.what());
  }

  // Topics to publish
  m_pub_markers = m_nh.advertise<rnrt_msgs::Markers>(m_marker_pose_topic, 1);
  m_pub_image = m_nh.advertise<sensor_msgs::Image>(m_output_image_topic, 1);

  // Topic to subscribe
  readDetectorParams();
  readCameraParams();

  m_sub = m_nh.subscribe(m_source_camera_topic, 1, &MarkerDetector::callback, this);

  m_marker_ids.reserve(32);
  m_marker_corners.reserve(32);
  m_rvecs.reserve(32);
  m_tvecs.reserve(32);
}

MarkerDetector::~MarkerDetector()
{
}

void MarkerDetector::initParameters()
{
  m_nh.getParam(m_node_name + "/package_path", m_package_path);
  m_nh.getParam(m_node_name + "/camera_parameters_file", m_camera_parameters_file);
  m_nh.getParam(m_node_name + "/detector_parameters_file", m_detector_parameters_file);
  m_nh.getParam(m_node_name + "/source_camera_topic", m_source_camera_topic);
  m_nh.getParam(m_node_name + "/output_image_topic", m_output_image_topic);
  m_nh.getParam(m_node_name + "/marker_pose_topic", m_marker_pose_topic);
  m_nh.getParam(m_node_name + "/tf_parent_frame", m_tf_parent_frame);
  m_nh.getParam(m_node_name + "/tf_child_frame_prefix", m_tf_child_frame_prefix);
  m_nh.getParam(m_node_name + "/aruco_dict_type", m_aruco_dict_type);
  m_nh.getParam(m_node_name + "/aruco_marker_size", m_marker_size);
}

void MarkerDetector::initArucoDictSelector()
{
  m_aruco_dict_selector = {
    { "DICT_4X4_50", cv::aruco::DICT_4X4_50 },   { "DICT_4X4_100", cv::aruco::DICT_4X4_100 },
    { "DICT_4X4_250", cv::aruco::DICT_4X4_250 }, { "DICT_4X4_1000", cv::aruco::DICT_4X4_1000 },

    { "DICT_5X5_50", cv::aruco::DICT_5X5_50 },   { "DICT_5X5_100", cv::aruco::DICT_5X5_100 },
    { "DICT_5X5_250", cv::aruco::DICT_5X5_250 }, { "DICT_5X5_1000", cv::aruco::DICT_5X5_1000 },

    { "DICT_6X6_50", cv::aruco::DICT_6X6_50 },   { "DICT_6X6_100", cv::aruco::DICT_6X6_100 },
    { "DICT_6X6_250", cv::aruco::DICT_6X6_250 }, { "DICT_6X6_1000", cv::aruco::DICT_6X6_1000 },

    { "DICT_7X7_50", cv::aruco::DICT_7X7_50 },   { "DICT_7X7_100", cv::aruco::DICT_7X7_100 },
    { "DICT_7X7_250", cv::aruco::DICT_7X7_250 }, { "DICT_7X7_1000", cv::aruco::DICT_7X7_1000 }
  };
}

void MarkerDetector::readCameraParams()
{
  cv::FileStorage fs(m_package_path + m_camera_parameters_file, cv::FileStorage::READ);
  fs["camera_matrix"] >> m_camera_matrix;
  fs["distortion_coefficients"] >> m_dist_coeffs;
}

void MarkerDetector::readDetectorParams()
{
  m_detector_parameters = cv::aruco::DetectorParameters::create();
  cv::FileStorage fs(m_package_path + m_detector_parameters_file, cv::FileStorage::READ);
  fs["adaptiveThreshWinSizeMin"] >> m_detector_parameters->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> m_detector_parameters->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> m_detector_parameters->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> m_detector_parameters->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> m_detector_parameters->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> m_detector_parameters->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> m_detector_parameters->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> m_detector_parameters->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> m_detector_parameters->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> m_detector_parameters->minMarkerDistanceRate;
  fs["cornerRefinementWinSize"] >> m_detector_parameters->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> m_detector_parameters->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> m_detector_parameters->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> m_detector_parameters->markerBorderBits;

  fs["perspectiveRemovePixelPerCell"] >> m_detector_parameters->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> m_detector_parameters->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> m_detector_parameters->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> m_detector_parameters->minOtsuStdDev;
  fs["errorCorrectionRate"] >> m_detector_parameters->errorCorrectionRate;

  m_detector_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

geometry_msgs::Quaternion MarkerDetector::rotMatToQuat(const cv::Mat& rot) const
{
  tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), rot.at<double>(1, 0),
                         rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
                         rot.at<double>(2, 2));

  tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());
  geometry_msgs::Pose pose_msg;
  tf2::toMsg(tf2_transform, pose_msg);
  return pose_msg.orientation;
}

geometry_msgs::TransformStamped MarkerDetector::makeTransformMsg(geometry_msgs::Pose& pose, const int& marker_id) const
{
  geometry_msgs::TransformStamped result;

  result.header.stamp = ros::Time::now();
  result.header.frame_id = m_tf_parent_frame;
  result.child_frame_id = m_tf_child_frame_prefix + std::to_string(marker_id);
  result.transform.translation.x = pose.position.x;
  result.transform.translation.y = pose.position.y;
  result.transform.translation.z = pose.position.z;
  result.transform.rotation.x = pose.orientation.x;
  result.transform.rotation.y = pose.orientation.y;
  result.transform.rotation.z = pose.orientation.z;
  result.transform.rotation.w = pose.orientation.w;

  return result;
}

void MarkerDetector::callback(const sensor_msgs::Image::ConstPtr& img) const
{
  m_marker_ids.clear();
  m_marker_corners.clear();
  m_rvecs.clear();
  m_tvecs.clear();

  static tf2_ros::TransformBroadcaster tf_broadcaster;

  rnrt_msgs::Markers markers_msg;
  geometry_msgs::Pose marker_pose;

  sensor_msgs::Image image_msg;

  auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

  cv::Mat image_copy;
  (cv_ptr->image).copyTo(image_copy);

  cv::aruco::detectMarkers(image_copy, m_dict, m_marker_corners, m_marker_ids, m_detector_parameters);

  cv::aruco::drawDetectedMarkers(image_copy, m_marker_corners, m_marker_ids);

  cv::aruco::estimatePoseSingleMarkers(m_marker_corners, m_marker_size, m_camera_matrix, m_dist_coeffs, m_rvecs,
                                       m_tvecs);

  // Draw axis for each marker
  for (auto i{ 0 }; i < m_marker_ids.size(); i++)
  {
    cv::aruco::drawAxis(image_copy, m_camera_matrix, m_dist_coeffs, m_rvecs[i], m_tvecs[i], 0.05);
  }

  for (auto i{ 0 }; i < m_marker_ids.size(); i++)
  {
    markers_msg.marker_ids.push_back(m_marker_ids[i]);
    marker_pose = makeMarkerPose(m_marker_ids[i], m_rvecs[i], m_tvecs[i]);
    markers_msg.poses.push_back(marker_pose);

    tf_broadcaster.sendTransform(makeTransformMsg(marker_pose, m_marker_ids[i]));
  }

  m_pub_markers.publish(markers_msg);

  cv_ptr->image = image_copy;
  (*cv_ptr).toImageMsg(image_msg);
  m_pub_image.publish(image_msg);
}

geometry_msgs::Pose MarkerDetector::makeMarkerPose(const int& marker_id, const cv::Vec3d& rvec,
                                                   const cv::Vec3d& tvec) const
{
  geometry_msgs::Pose marker_pose;
  geometry_msgs::Point marker_point;
  cv::Mat rotation_matrix;

  marker_point.x = tvec[0];
  marker_point.y = tvec[1];
  marker_point.z = tvec[2];

  cv::Rodrigues(rvec, rotation_matrix);

  marker_pose.position = marker_point;
  marker_pose.orientation = rotMatToQuat(rotation_matrix);

  return marker_pose;
}
