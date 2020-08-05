#include <iostream>
#include <ros/package.h>
#include "sr_hand_detector/sr_hand_detector.h"
#include "sr_hand_detector/sr_hand_autodetect.h"

int main(int argc, char* argv[])
{
  sr_hand_detector::SrHandDetector sr_hand_detector;
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(sr_hand_detector);
  sr_hand_autodetect.run();

  std::string command_string = "";
  for (int i = 1; i < argc; i++)
  {
      command_string += argv[i];
      if (!(argc - 1 == i))
      {
          command_string += " ";
      }
  }

    command_string += sr_hand_autodetect.command_sufix;

    std::cout << command_string << std::endl;
    system(command_string.c_str());
    return 0;
}