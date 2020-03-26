/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include "ros/ros.h"
#include "sr_hand_detector/sr_hand_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_hand_detector");

  sr_hand_detector::SrHandDetector sr_hand_detector;
  sr_hand_detector.get_port_names();
  for (int i=0; i<sr_hand_detector.num_ports_; i++)
  {
      ROS_INFO_STREAM(sr_hand_detector.port_names_[i] << std::endl);
      ROS_INFO_STREAM(sr_hand_detector.count_slaves(i) << std::endl);
  }
  return 0;
}