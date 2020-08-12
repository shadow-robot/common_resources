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

#include "sr_hand_detector/sr_hand_detector.h"
#include <string>
#include <map>
#include <utility>
#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv)
{
  sr_hand_detector::SrHandDetector sr_hand_detector;
  sr_hand_detector.run();

  if (sr_hand_detector.hand_serial_and_port_map_.empty())
  {
    std::cout << "No hand detected on any of the ports!" << std::endl;
  }

  YAML::Node hand_serial_to_port_map;
  for (auto const& x : sr_hand_detector.hand_serial_and_port_map_)
  {
    std::cout << "Detected hand on port: " << x.second << std::endl;
    std::cout << "Hand's serial number: " << x.first << std::endl;
    hand_serial_to_port_map[x.first] = x.second;
  }

  std::ofstream fout("/tmp/sr_hand_detector.yaml");
  fout << hand_serial_to_port_map;
  return 0;
}
