/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
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

    std::map<std::string, int> hand_port_and_serial_map_;
};

}  // namespace sr_hand_detector

#endif  //  SR_HAND_DETECTOR_SR_HAND_DETECTOR_H