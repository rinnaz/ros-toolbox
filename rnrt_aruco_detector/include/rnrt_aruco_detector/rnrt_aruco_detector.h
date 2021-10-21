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
    void readCameraParams();
    void readDetectorParams();
    void callback(const sensor_msgs::Image::ConstPtr& img) const;
    geometry_msgs::Quaternion rotMatToQuat(const cv::Mat& rot_mat) const;
    geometry_msgs::TransformStamped makeTransformMsg(geometry_msgs::Pose& pose) const;

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub_pose, m_pub_image;
    ros::Subscriber m_sub;

    const std::string m_package_path;
    const std::string m_cameraParamsFile;
    const std::string m_detectorParamsFile;
    const std::string m_cameraTopicName;
    const std::string m_imageTopicName;
    const std::string m_markersTopicName;

    cv::Ptr<cv::aruco::DetectorParameters> m_detectorParams;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> m_dict;
};