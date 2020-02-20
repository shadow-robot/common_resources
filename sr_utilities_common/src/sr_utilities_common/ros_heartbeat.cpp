/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <sr_utilities_common/ros_heartbeat.h>


RosHeartbeat::RosHeartbeat(std::string heartbeat_topic_name):
  heartbeat_topic_subscriber_(nh_.subscribe(heartbeat_topic_name, 1,
                             &RosHeartbeat::on_heartbeat_message_cb, this))
{
  ROS_INFO("Initializing heartbeat...");
}

RosHeartbeat::~RosHeartbeat()
{
}

void RosHeartbeat::on_heartbeat_message_cb(const std_msgs::Bool& heartbeat)
{
  bool result = heartbeat.data;
  ROS_INFO_STREAM(result);
}