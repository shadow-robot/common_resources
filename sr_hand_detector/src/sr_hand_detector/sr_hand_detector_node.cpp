/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include "ros/ros.h"
#include "sr_hand_detector/sr_hand_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_hand_detector");

  sr_hand_detector::SrHandDetector sr_hand_detector();
  return 0;
}