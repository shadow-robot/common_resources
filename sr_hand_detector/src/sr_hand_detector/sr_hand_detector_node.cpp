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
  sr_hand_detector.run();

  for (auto const& x : sr_hand_detector.hand_port_and_serial_map_)
  {
    ROS_INFO_STREAM("Detected hand on port: " << x.first);
    ROS_INFO_STREAM("Hand's serial number: " << x.second);
  }

  return 0;
}