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

#include "ros/ros.h"
#include "sr_hand_detector/sr_hand_detector.h"
#include <string>
#include <map>
#include <utility>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_hand_detector");
  ros::NodeHandle nh;

  sr_hand_detector::SrHandDetector sr_hand_detector;
  sr_hand_detector.run();

  if (sr_hand_detector.hand_serial_and_port_map_.empty())
  {
    ROS_WARN_STREAM("No hand detected on any of the ports!");
    return 1;
  }

  std::map<std::string, std::string> hand_serial_and_port_map_strings;
  for (auto const& x : sr_hand_detector.hand_serial_and_port_map_)
  {
    ROS_INFO_STREAM("Detected hand on port: " << x.second);
    ROS_INFO_STREAM("Hand's serial number: " << x.first);

    hand_serial_and_port_map_strings.insert(std::pair<std::string, std::string>(std::to_string(x.first),
                                                                                x.second));
  }

  nh.setParam("hand/eth_port", hand_serial_and_port_map_strings);

  return 0;
}
