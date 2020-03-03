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
    RosHeartbeat(std::string, float);
    ~RosHeartbeat();
    bool enabled;

  private:
    bool heartbeat_detected_= true;
    ros::Duration heartbeat_timer_duration_;
    ros::NodeHandle nh_ = ros::NodeHandle();
    ros::Subscriber heartbeat_topic_subscriber_;
    ros::Timer heartbeat_timer_;

    void on_heartbeat_message_cb(const std_msgs::Bool&);
    void on_heartbeat_absent(const ros::TimerEvent&);
};

#endif  //  ROS_HEARTBEAT_ROS_HEARTBEAT_H
