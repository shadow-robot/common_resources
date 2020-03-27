/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef SR_HAND_DETECTOR_SR_HAND_DETECTOR_H
#define SR_HAND_DETECTOR_SR_HAND_DETECTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <vector>
#include <map>
#include <string>
#include "soem/ethercattype.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatcoe.h"

#define MAXBUF 32768

namespace sr_hand_detector
{
class SrHandDetector
{
//   private:

  public:
    SrHandDetector();
    ~SrHandDetector();
    void find_slaves();
    void get_port_names();
    int count_slaves_on_port(std::string);
    void detect_hand_ports();
    int get_hand_serial(std::string);
    int read_eeprom(int, int, int);

    std::vector<std::string> available_port_names_;
    std::vector<std::string> hand_port_names_;
    uint8 ebuf[MAXBUF];
};

}  // namespace sr_hand_detector

#endif  //  SR_HAND_DETECTOR_SR_HAND_DETECTOR_H