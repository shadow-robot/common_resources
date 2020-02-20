/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef ROS_HEARTBEAT_ROS_HEARTBEAT_H
#define ROS_HEARTBEAT_ROS_HEARTBEAT_H

#include "ros/ros.h"

class RosHeartbeat
{
  public:
    RosHeartbeat(std::string input_topic_name);
    ~RosHeartbeat();
    std::string tmp;

  private:
    std::string input_topic_name_;
};

#endif  //  ROS_HEARTBEAT_ROS_HEARTBEAT_H
