/*
* Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H
#define SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H

#include <ros/ros.h>

bool wait_for_param(ros::NodeHandle node_handle, std::string param_name, double timeout_in_secs = 0);

#endif  //  SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H
