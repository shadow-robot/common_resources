#!/usr/bin/env python3

# Copyright 2020, 2022 Shadow Robot Company Ltd.
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

import os
import re
import sys
import time
import rosservice
import rospy
from sr_utilities_common.wait_for_param import wait_for_param


class RosElementsHandler:
    def __init__(self, element_type, required_elements_list):
        self._element_type = element_type
        # De-duplicate and make a copy of the required elements list
        self.missing_elements = list(set(required_elements_list))
        self._verify_elements()
        self._strip_elements_of_leading_slash_if_present()

    def check_if_required_element_is_available(self):
        roscore_published_elements = self._retrieve_available_elements()
        # If there are missing elements, try to find them, else return true
        if self.missing_elements:
            # Loop through a copy of missing elements; we're removing items from it within the loop
            for missing_element in list(self.missing_elements):
                for element in roscore_published_elements:
                    if re.match(missing_element, element):
                        rospy.loginfo("%s: Found %s", rospy.get_name(), missing_element)
                        self.missing_elements.remove(missing_element)
                        break

            # Return true if there are no more missing elements
            return not self.missing_elements
        return True

    def _verify_elements(self):
        for element in list(self.missing_elements):
            if not (element and isinstance(element, str)):
                raise ValueError("{}: Required element is not a string or an empty string".format(rospy.get_name()))

    def _strip_elements_of_leading_slash_if_present(self):
        for idx, element in enumerate(self.missing_elements):
            if element.startswith("/"):
                self.missing_elements[idx] = element[1:]

    def _retrieve_available_elements(self):
        if self._element_type == "topic":
            list_of_topics = [item[0][1:] for item in rospy.get_published_topics()]
            return list_of_topics
        if self._element_type == "service":
            return [item[1:] for item in rosservice.get_service_list()]
        if self._element_type == "param":
            return [item[1:] for item in rospy.get_param_names()]

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
        if rospy.Time.now().to_sec() - start_time.to_sec() >= timeout:
            rospy.logerr("Timeout of {}s exceeded".format(timeout))
            for condition_type, condition in conditions_to_satisfy.items():
                if condition.missing_elements:
                    rospy.logerr('Could not find the following {}s: {}'.format(condition_type,
                                                                               condition.missing_elements))
            break
    return False


if __name__ == "__main__":
    rospy.init_node('conditional_delayed_roslaunch', anonymous=True)

    conditions_to_satisfy_arg = {}

    package_name = rospy.get_param("~package_name")
    executable_name = rospy.get_param("~executable_name")
    arguments_list = rospy.get_param("~launch_args_list", "")
    node_launch_prefix = rospy.get_param("~launch_prefix", "")
    timeout_param = rospy.get_param("~timeout", 0)

    if rospy.has_param('~topics_list'):
        topics_list = rospy.get_param("~topics_list")
        conditions_to_satisfy_arg["topic"] = RosElementsHandler("topic", topics_list)
    if rospy.has_param('~params_list'):
        params_list = rospy.get_param("~params_list")
        conditions_to_satisfy_arg["param"] = RosElementsHandler("param", params_list)
    if rospy.has_param('~services_list'):
        services_list = rospy.get_param("~services_list")
        conditions_to_satisfy_arg["service"] = RosElementsHandler("service", services_list)

    all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy_arg, timeout_param)

    if rospy.has_param('~args_from_param_list'):
        args_from_param_list = rospy.get_param("~args_from_param_list")
        for arg_from_param in args_from_param_list:
            if arg_from_param.startswith("/"):
                arg_from_param = arg_from_param[1:]
            all_conditions_satisfied = all_conditions_satisfied and wait_for_param(arg_from_param, timeout_param)
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
