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
  sr_hand_detector.get_available_port_names();
  sr_hand_detector.detect_hand_ports();
  for (int i=0; i<sr_hand_detector.hand_port_names_.size(); i++)
  {
    // std::cout << sr_hand_detector.available_ports_names_[i] << std::endl;
    ROS_INFO_STREAM("Detected hand on port: " << sr_hand_detector.hand_port_names_[i]);
    ROS_INFO_STREAM("Hand's serial number: " << sr_hand_detector.get_hand_serial(sr_hand_detector.hand_port_names_[i]));
  }
  return 0;
}