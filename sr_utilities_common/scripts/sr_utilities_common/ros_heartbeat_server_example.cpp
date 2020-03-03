/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <sr_utilities_common/ros_heartbeat.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_heartbeat_example");
  RosHeartbeat ros_heartbeat("test_heartbeat", 0.1);
  ros::spin();
  return 0;
}
