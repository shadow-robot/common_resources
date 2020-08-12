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

#include "sr_hand_detector/sr_hand_autodetect.h"
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"
#include <iostream>
#include <ros/package.h>
#include <string>
#include <utility>

namespace sr_hand_detector
{
SrHandAutodetect::SrHandAutodetect(std::string detected_hands_file, std::string hand_config_path) :
  detected_hands_file_(detected_hands_file)
{
  if (hand_config_path.empty())
  {
    get_path_to_sr_hand_config();
  }
  else
  {
    sr_hand_config_path_ = hand_config_path;
  }
}

SrHandAutodetect::~SrHandAutodetect()
{
}

void SrHandAutodetect::get_path_to_sr_hand_config()
{
  sr_hand_config_path_ = ros::package::getPath("sr_hand_config");
  if (sr_hand_config_path_.empty())
  {
    throw std::runtime_error("sr_hand_autodetect: Did not find sr_hand_config package.");
  }
}

YAML::Node SrHandAutodetect::get_hand_general_info(int serial)
{
  std::string path_to_info_file = sr_hand_config_path_ + "/" + std::to_string(serial) + "/general_info.yaml";
  try
  {
    return YAML::LoadFile(path_to_info_file);
  }
  catch(YAML::BadFile)
  {
    std::cerr << "sr_hand_autodetect: General info for detected hand does not exist!";
    throw;
  }
}

std::string SrHandAutodetect::get_hand_id(std::string hand_side)
{
  if ("right" == hand_side)
  {
    return "rh";
  }
  else if ("left" == hand_side)
  {
    return "lh";
  }
  else
  {
    throw std::runtime_error("sr_hand_autodetect: Unknown hand side.");
  }
}

void SrHandAutodetect::detect_hands()
{
  YAML::Node config = YAML::LoadFile(detected_hands_file_);
  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
  {
    hand_serial_and_port_map_.insert(std::pair<int, std::string>(it->first.as<int>(), it->second.as<std::string>()));
  }
  number_of_detected_hands_ = hand_serial_and_port_map_.size();
}

void SrHandAutodetect::compose_command_suffix()
{
  if (0 == number_of_detected_hands_)
  {
    std::cout << "No hands detected. Not wrapping the roslaunch command!";
    command_sufix_ = "";
  }
  else if (1 == number_of_detected_hands_)
  {
    int hand_serial = hand_serial_and_port_map_.begin()->first;
    std::string eth_port = hand_serial_and_port_map_.begin()->second;
    YAML::Node hand_info = get_hand_general_info(hand_serial);
    std::string hand_id = get_hand_id(hand_info["side"].as<std::string>());

    command_sufix_ = " eth_port:=" + eth_port + " hand_serial:=" +
      std::to_string(hand_serial) + " hand_id:=" + hand_id;
  }
  else if (2 == number_of_detected_hands_)
  {
    int rh_serial, lh_serial;
    std::string rh_eth_port, lh_eth_port;

    for (auto const& serial_to_port : hand_serial_and_port_map_)
    {
      YAML::Node hand_info = get_hand_general_info(serial_to_port.first);
      std::string hand_id = get_hand_id(hand_info["side"].as<std::string>());
      if ("rh" == hand_id)
      {
        rh_serial = serial_to_port.first;
        rh_eth_port = serial_to_port.second;
      }
      else if ("lh" == hand_id)
      {
        lh_serial = serial_to_port.first;
        lh_eth_port = serial_to_port.second;
      }
      else
      {
        throw std::runtime_error("sr_hand_autodetect: Unsupported hand id");
      }
    }

    command_sufix_ = " eth_port:=" + rh_eth_port + "_" + lh_eth_port + " rh_serial:=" +
      std::to_string(rh_serial) + " lh_serial:=" + std::to_string(lh_serial);
  }
  else
  {
    throw std::runtime_error("sr_hand_autodetect: Unsupported number of hands detected in the system");
  }
}

void SrHandAutodetect::run()
{
  detect_hands();
  compose_command_suffix();
}
}  // namespace sr_hand_detector
