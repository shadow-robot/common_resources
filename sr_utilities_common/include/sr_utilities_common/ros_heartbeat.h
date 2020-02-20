/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef ROS_HEARTBEAT_ROS_HEARTBEAT_H
#define ROS_HEARTBEAT_ROS_HEARTBEAT_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class RosHeartbeat
{
  public:
    RosHeartbeat(std::string heartbeat_topic_name);
    ~RosHeartbeat();
    bool enable;

  private:
    ros::NodeHandle nh_ = ros::NodeHandle();
    ros::Subscriber heartbeat_topic_subscriber_;

    void on_heartbeat_message_cb(const std_msgs::Bool&); 
};

#endif  //  ROS_HEARTBEAT_ROS_HEARTBEAT_H
