
/*
* Copyright 2020 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SR_UTILITIES_COMMON_ROS_HEARTBEAT_H
#define SR_UTILITIES_COMMON_ROS_HEARTBEAT_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

class RosHeartbeat
{
  public:
    RosHeartbeat(std::string, float);
    ~RosHeartbeat();
    bool enabled;

  private:
    bool heartbeat_detected_ = true;
    ros::Duration heartbeat_timer_duration_;
    ros::NodeHandle nh_ = ros::NodeHandle();
    ros::Subscriber heartbeat_topic_subscriber_;
    ros::Timer heartbeat_timer_;

    void on_heartbeat_message_cb(const std_msgs::Bool&);
    void on_heartbeat_absent(const ros::TimerEvent&);
};

#endif  //  SR_UTILITIES_COMMON_ROS_HEARTBEAT_H
