#include <iostream>
#include <ros/package.h>
#include "sr_hand_detector/sr_hand_detector.h"
#include "sr_hand_detector/sr_hand_autodetect.h"

int main(int argc, char* argv[])
{
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect;
  sr_hand_autodetect.run();

    sr_hand_detector::SrHandDetector sr_hand_detector;
    sr_hand_detector.run();
    int number_of_hands_detected;

  std::string sr_hand_config_path = ros::package::getPath("sr_hand_config");
  std::cout << sr_hand_config_path << std::endl;
  number_of_hands_detected = sr_hand_detector.hand_serial_and_port_map_.size();

  if (1 != number_of_hands_detected | 2 != number_of_hands_detected)
  {
    throw std::runtime_error("sr_hand_autodetect: Unsupported number of hands detected in the system");
  }

  std::map<std::string, std::string> hand_serial_and_port_map_strings;
  for (auto const& x : sr_hand_detector.hand_serial_and_port_map_)
  {
    hand_serial_and_port_map_strings.insert(std::pair<std::string, std::string>(std::to_string(x.first),
                                                                                x.second));
  }

  if (1 == number_of_hands_detected)
  {
    
  }
  else if (2 == number_of_hands_detected)
  {
    ;
  }

    std::string command_string = "";
    for (int i = 1; i < argc; i++)
    {
        command_string += argv[i];
        if (!(argc - 1 == i))
        {
            command_string += " ";
        }
    }

    command_string += " sim:=false";

    std::cout << command_string << std::endl;
    system(command_string.c_str());
    return 0;
}