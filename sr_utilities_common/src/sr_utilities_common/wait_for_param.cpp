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

#include <sr_utilities_common/wait_for_param.h>
#include <string>

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
      ros::Duration(0.01).sleep();
    }
    return false;
  }

  while (ros::Time::now().toSec() - start_time.toSec() < timeout_in_secs)
  {
    if (node_handle.hasParam(param_name))
    {
      return true;
    }
    ros::Duration(0.1).sleep();
  }
  return false;
}
