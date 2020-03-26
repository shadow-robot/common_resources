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


namespace sr_hand_detector
{
class SrHandDetector
{
  public:
    SrHandDetector();
    ~SrHandDetector();
};

}  // namespace sr_hand_detector

#endif  //  SR_HAND_DETECTOR_SR_HAND_DETECTOR_H