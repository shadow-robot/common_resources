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

#include <iostream>
#include <string>
#include <ros/package.h>
#include "sr_hand_detector/sr_hand_autodetect.h"

int main(int argc, char* argv[])
{
  system("sr_hand_detector_node");

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect;
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

  command_string += sr_hand_autodetect.command_sufix_;
  std::cout << "Actual command run: " << command_string << std::endl;
  system(command_string.c_str());
  return 0;
}
