#include "rnrt_aruco_detector/rnrt_aruco_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_detector");

  ROS_WARN("aruco_detector process started");

  MarkerDetector *detector = new MarkerDetector();

  ros::spin();
  return 0;
}
 
