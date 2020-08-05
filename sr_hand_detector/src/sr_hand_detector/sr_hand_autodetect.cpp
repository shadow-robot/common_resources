#include "sr_hand_detector/sr_hand_autodetect.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ros/package.h>

namespace sr_hand_detector
{
SrHandAutodetect::SrHandAutodetect(SrHandDetector sr_hand_detector, std::string hand_config_path) :
  sr_hand_detector_(sr_hand_detector)
{
  if (hand_config_path.empty())
  {
    get_path_to_sr_hand_config();
  }
  else
  {
    sr_hand_config_path = hand_config_path;
  }
}

SrHandAutodetect::~SrHandAutodetect()
{
}

void SrHandAutodetect::get_path_to_sr_hand_config()
{
  sr_hand_config_path = ros::package::getPath("sr_hand_config");
}

YAML::Node SrHandAutodetect::get_hand_general_info(int serial)
{
  std::string path_to_info_file = sr_hand_config_path + "/" + std::to_string(serial) + "/general_info.yaml";
  return YAML::LoadFile(path_to_info_file);
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
  sr_hand_detector_.run();
  hand_serial_and_port_map = sr_hand_detector_.hand_serial_and_port_map_;
  number_of_detected_hands = hand_serial_and_port_map.size();
}

void SrHandAutodetect::compose_command_sufix()
{
  if (0 == number_of_detected_hands)
  {
    std::cout << "No hands detected. Not wrapping the roslaunch command!";
    command_sufix = "";
  }
  else if (1 == number_of_detected_hands)
  {
    int hand_serial = hand_serial_and_port_map.begin()->first;
    std::string eth_port = hand_serial_and_port_map.begin()->second;
    YAML::Node hand_info = get_hand_general_info(hand_serial);
    std::string hand_id = get_hand_id(hand_info["side"].as<std::string>());

    command_sufix = " eth_port:=" + eth_port + " hand_serial:=" + std::to_string(hand_serial) + " hand_id:=" + hand_id;
  }
  else if (2 == number_of_detected_hands)
  {
    int rh_serial, lh_serial;
    std::string rh_eth_port, lh_eth_port;

    for (auto const& serial_to_port : hand_serial_and_port_map)
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

    command_sufix = " eth_port:=" + rh_eth_port + "_" + lh_eth_port + " rh_serial:=" + std::to_string(rh_serial) + " lh_serial:=" + std::to_string(lh_serial);
  }
  else
  {
    throw std::runtime_error("sr_hand_autodetect: Unsupported number of hands detected in the system");
  }

}

void SrHandAutodetect::run()
{
  detect_hands();
  compose_command_sufix();
}
}