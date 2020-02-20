/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include "sr_utilities_common/ros_heartbeat.h"


RosHeartbeat::RosHeartbeat(std::string input_topic_name)
{
  ROS_INFO("Initializing heartbeat...");
  input_topic_name_ = input_topic_name;
  tmp = input_topic_name;
}

RosHeartbeat::~RosHeartbeat()
{
}
