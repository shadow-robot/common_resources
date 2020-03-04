/*
* Copyright <Year> Shadow Robot Company Ltd.
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_heartbeat_example");
  RosHeartbeat ros_heartbeat("example_heartbeat", 0.1);
  ros::spin();
  return 0;
}
