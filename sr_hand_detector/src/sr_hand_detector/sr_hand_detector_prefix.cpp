#include <iostream>
#include "sr_hand_detector/sr_hand_detector.h"

int main(int argc, char* argv[])
{
    sr_hand_detector::SrHandDetector sr_hand_detector;
    sr_hand_detector.run();

  if (sr_hand_detector.hand_serial_and_port_map_.empty())
  {
    std::cout << "No hand detected on any of the ports!";
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