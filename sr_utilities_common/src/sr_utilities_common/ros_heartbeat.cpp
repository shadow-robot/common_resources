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

#include <sr_utilities_common/ros_heartbeat.h>
#include <string>

RosHeartbeat::RosHeartbeat(std::string heartbeat_topic_name, float heartbeat_timer_duration):
  heartbeat_timer_duration_(heartbeat_timer_duration)
{
  ROS_INFO_STREAM(ros::this_node::getName() << ": initializing heartbeat...");
  heartbeat_topic_subscriber_ = nh_.subscribe(heartbeat_topic_name, 1,
                                              &RosHeartbeat::on_heartbeat_message_cb, this);
  // let subscriber sort itself before starting the timer
  ros::Duration(1).sleep();
  heartbeat_timer_ = nh_.createTimer(heartbeat_timer_duration_, &RosHeartbeat::on_heartbeat_absent, this);
}

RosHeartbeat::~RosHeartbeat()
{
}

void RosHeartbeat::on_heartbeat_message_cb(const std_msgs::Bool& heartbeat)
{
  if (!heartbeat_detected_)
  {
    ROS_INFO_STREAM(ros::this_node::getName() << ": heartbeat detected.");
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
