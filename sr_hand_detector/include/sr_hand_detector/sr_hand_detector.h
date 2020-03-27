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
#include "soem/ethercattype.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatcoe.h"

#define MAX_PORTS 8
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
    int count_slaves(int);
    void add_port_name(char*);
    void detect_hand_ports();
    int get_hand_serial(char*);
    int read_eeprom(int, int, int);

    char* available_port_names_[MAX_PORTS];
    char* hand_port_names_[MAX_PORTS];
    uint8 ebuf[MAXBUF];
    int num_ports_ = 0;
    int num_hands_= 0;
};

}  // namespace sr_hand_detector

#endif  //  SR_HAND_DETECTOR_SR_HAND_DETECTOR_H