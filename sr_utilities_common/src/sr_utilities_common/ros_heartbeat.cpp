/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <sr_utilities_common/ros_heartbeat.h>
#include <string>

RosHeartbeat::RosHeartbeat(std::string heartbeat_topic_name, float heartbeat_timer_duration):
  heartbeat_timer_duration_(heartbeat_timer_duration)
{
  ROS_INFO_STREAM(ros::this_node::getName() << ": initializing heartbeat...");
  heartbeat_topic_subscriber_ = nh_.subscribe(heartbeat_topic_name, 1,
                                              &RosHeartbeat::on_heartbeat_message_cb, this);
  ros::Duration(1).sleep();  // let subscriber sort itself before starting the timer
  heartbeat_timer_ = nh_.createTimer(heartbeat_timer_duration_, &RosHeartbeat::on_heartbeat_absent, this);
}

RosHeartbeat::~RosHeartbeat()
{
}

void RosHeartbeat::on_heartbeat_message_cb(const std_msgs::Bool& heartbeat)
{
  if (!heartbeat_detected_)
  {
    ROS_INFO_STREAM(ros::this_node::getName() << ": heartbeat detected again!");
    heartbeat_detected_ = true;
  }
  heartbeat_timer_.setPeriod(heartbeat_timer_duration_, true);
  heartbeat_timer_.start();
  enabled = heartbeat.data;
}

void RosHeartbeat::on_heartbeat_absent(const ros::TimerEvent&)
{
    ROS_WARN_STREAM(ros::this_node::getName() << ": heartbeat not detected!");
    heartbeat_timer_.stop();
    heartbeat_detected_ = false;
    enabled = false;
}
