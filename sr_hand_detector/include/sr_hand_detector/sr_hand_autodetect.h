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

#ifndef SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H
#define SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H

#include <string>
#include <map>
#include "sr_hand_detector/sr_hand_detector.h"
#include "yaml-cpp/yaml.h"

namespace sr_hand_detector
{
class SrHandAutodetect
{
  void get_path_to_sr_hand_config();
  YAML::Node get_hand_general_info(int serial);
  void detect_hands();
  void compose_command_suffix();
  std::string get_hand_id(std::string hand_side);

  int number_of_detected_hands_;
  std::string sr_hand_config_path_;
  std::string detected_hands_file_;
  std::map<int, std::string> hand_serial_and_port_map_;

  public:
    explicit SrHandAutodetect(std::string detected_hands_file = "/tmp/sr_hand_detector.yaml",
                              std::string hand_config_path = "");
    ~SrHandAutodetect();
    void run();

    std::string command_sufix_;
};

}  // namespace sr_hand_detector

#endif  // SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H
