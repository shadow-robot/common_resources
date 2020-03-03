/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <sr_utilities_common/ros_heartbeat.h>


RosHeartbeat::RosHeartbeat(std::string heartbeat_topic_name):
  heartbeat_topic_subscriber_(nh_.subscribe(heartbeat_topic_name, 1,
                             &RosHeartbeat::on_heartbeat_message_cb, this)),
  heartbeat_timer_(nh_.createTimer(ros::Duration(0.1), &RosHeartbeat::on_heartbeat_absent, this))
{
  ROS_INFO("Initializing heartbeat...");
}

RosHeartbeat::~RosHeartbeat()
{
}

void RosHeartbeat::on_heartbeat_message_cb(const std_msgs::Bool& heartbeat)
{
  heartbeat_timer_.setPeriod(ros::Duration(0.1), true);
  heartbeat_timer_.start();
  enable = heartbeat.data;
  ROS_INFO_STREAM(enable);
}

void RosHeartbeat::on_heartbeat_absent(const ros::TimerEvent&)
{
    ROS_WARN("timer");
    enable = false;
    heartbeat_timer_.stop();
}