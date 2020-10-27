#include <sr_utilities_common/wait_for_param.h>

bool wait_for_param(ros::NodeHandle node_handle, std::string param_name, double timeout_in_secs)
{
  ros::Time start_time = ros::Time::now();
  if (timeout_in_secs <= 0)
  {
    const double TIME_BEFORE_INFO = 60;
    while (ros::ok())
    {
      if (node_handle.hasParam(param_name))
      {
        return true;
      }
      if (ros::Time::now().toSec() - start_time.toSec() >= TIME_BEFORE_INFO)
      {
        ROS_INFO_STREAM("Still waiting for parameter: " << param_name);
        start_time = ros::Time::now();
      }
    }
    return false;
  }

  while (ros::Time::now().toSec() - start_time.toSec() < timeout_in_secs)
  {
    if (node_handle.hasParam(param_name))
    {
      return true;
    }
  }
  return false;
}