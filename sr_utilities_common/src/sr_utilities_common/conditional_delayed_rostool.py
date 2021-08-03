#!/usr/bin/env python3

# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
from builtins import input
import rospy
import os
import rosservice
import sys
import time
from multiprocessing import Process
from sr_utilities_common.wait_for_param import wait_for_param


class RosElementsHandler(object):
    def __init__(self, element_type, required_elements_list):
        self._element_type = element_type
        # De-duplicate and make a copy of the required elements list
        self.missing_elements = list(set(required_elements_list))

    def check_if_required_element_is_available(self):
        roscore_published_elements = self._retrieve_available_elements()
        # If there are missing elements, try to find them, else return true
        if self.missing_elements:
            # Loop through a copy of missing elements; we're removing items from it within the loop
            for element in list(self.missing_elements):
                if element and type(element) == str:
                    if element in roscore_published_elements:
                        rospy.loginfo("%s: Found %s", rospy.get_name(), element)
                        self.missing_elements.remove(element)
                else:
                    raise ValueError("{}: Required element is not a string".format(rospy.get_name()))
            # Return true if there are no more missing elements
            return not self.missing_elements
        else:
            return True

    def _retrieve_available_elements(self):
        if self._element_type == "topic":
            list_of_topics = [item[0] for item in rospy.get_published_topics()]
            return list_of_topics
        elif self._element_type == "service":
            return rosservice.get_service_list()
        elif self._element_type == "param":
            return rospy.get_param_names()
        else:
            rospy.logerr("Requested ros element %s does not exist", self._element_type)
            return None


def wait_for_conditions(conditions_to_satisfy, timeout):
    start_time = rospy.Time.now()

    while True:
        all_conditions_satisfied_list = []
        for condition in list(conditions_to_satisfy.values()):
            all_conditions_satisfied_list.append(condition.check_if_required_element_is_available())
        if all(satisfied for satisfied in all_conditions_satisfied_list):
            return True
        time.sleep(0.01)
        if timeout <= 0:
            continue
        if (rospy.Time.now().to_sec() - start_time.to_sec() >= timeout):
            rospy.logerr("Timeout of {}s exceeded".format(timeout))
            for condition_type, condition in conditions_to_satisfy.items():
                if condition.missing_elements:
                    rospy.logerr('Could not find the following {}s: {}'.format(condition_type,
                                                                               condition.missing_elements))
            break
    return False


if __name__ == "__main__":
    rospy.init_node('conditional_delayed_roslaunch', anonymous=True)

    conditions_to_satisfy = {}

    package_name = rospy.get_param("~package_name")
    executable_name = rospy.get_param("~executable_name")
    arguments_list = rospy.get_param("~launch_args_list", "")
    node_launch_prefix = rospy.get_param("~launch_prefix", "")
    timeout = rospy.get_param("~timeout", 0)

    if rospy.has_param('~topics_list'):
        topics_list = rospy.get_param("~topics_list")
        conditions_to_satisfy["topic"] = RosElementsHandler("topic", topics_list)
    if rospy.has_param('~params_list'):
        params_list = rospy.get_param("~params_list")
        conditions_to_satisfy["param"] = RosElementsHandler("param", params_list)
    if rospy.has_param('~services_list'):
        services_list = rospy.get_param("~services_list")
        conditions_to_satisfy["service"] = RosElementsHandler("service", services_list)

    all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)

    if rospy.has_param('~args_from_param_list'):
        args_from_param_list = rospy.get_param("~args_from_param_list")
        for arg_from_param in args_from_param_list:
            all_conditions_satisfied = all_conditions_satisfied and wait_for_param(arg_from_param, timeout)
            if not all_conditions_satisfied:
                break
            arguments_list += " {}:={}".format(arg_from_param, rospy.get_param(arg_from_param))

    if all_conditions_satisfied:
        if executable_name.endswith('.launch'):
            os.system("roslaunch {} {} {}".format(package_name, executable_name, arguments_list))
        else:
            os.system("{} rosrun {} {} {}".format(node_launch_prefix, package_name, executable_name, arguments_list))
    else:
        rospy.logfatal("{}: Could not launch {} {}, make sure all required conditions are met".format(rospy.get_name(),
                                                                                                      package_name,
                                                                                                      executable_name))
        rospy.signal_shutdown("Required components missing")
        sys.exit(1)
