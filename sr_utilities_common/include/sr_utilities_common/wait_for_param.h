
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


#ifndef SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H
#define SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H

#include <ros/ros.h>
#include <string>

bool wait_for_param(ros::NodeHandle node_handle, std::string param_name, double timeout_in_secs = 0);

#endif  //  SR_UTILITIES_COMMON_WAIT_FOR_PARAM_H
