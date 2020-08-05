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

#ifndef SR_HAND_DETECTOR_SR_HAND_DETECTOR_H
#define SR_HAND_DETECTOR_SR_HAND_DETECTOR_H

#include <vector>
#include <map>
#include <string>
#include "soem/ethercattype.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"

#define MAXBUF 32768

namespace sr_hand_detector
{
class SrHandDetector
{
  void get_available_port_names();
  void detect_hand_ports();
  void get_hands_ports_and_serials();
  int count_slaves_on_port(std::string);
  int get_hand_serial(std::string);
  int read_eeprom(int, int, int);

  std::vector<std::string> available_port_names_;
  uint8 ebuf_[MAXBUF];
  const int SLAVE_WITH_HAND_SERIAL_ = 2;
  const int NUM_OF_SLAVES_EXPECTED_FOR_HAND_ = 2;

  public:
    SrHandDetector();
    ~SrHandDetector();
    void run();

    std::map<int, std::string> hand_serial_and_port_map_;
};

}  // namespace sr_hand_detector

#endif  // SR_HAND_DETECTOR_SR_HAND_DETECTOR_H
