#!/usr/bin/env python

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

import rospy
import os
import rosservice
import sys
from multiprocessing import Process


class RosElementsHandler(object):
    def __init__(self, element_type, required_elements_list):
        self._element_type = element_type
        self._required_elements_list = required_elements_list
        self._found_elements = []
        self.missing_elements = []

    def check_if_required_element_is_available(self):
        roscore_published_elements = self._retrieve_available_elements()
        if not isinstance(self._required_elements_list, list):
            raise ValueError("{}: Required elements must be a list not a string, \
                             check the required element list declaration".format(rospy.get_name()))
        if self._required_elements_list:
            for element in self._required_elements_list:
                if element and type(element) == str:
                    if any(element in sublist for sublist in roscore_published_elements):
                        rospy.loginfo("Found %s", element)
                        self._found_elements.append(element)
                else:
                    raise ValueError("{}: Required element is not a string".format(rospy.get_name()))
            if len(self._found_elements) == len(self._required_elements_list):
                return True
            self.missing_elements = list(set(self._required_elements_list) - set(self._found_elements))
        else:
            return True
        return False

    def _retrieve_available_elements(self):
        if self._element_type == "topic":
            return rospy.get_published_topics()
        elif self._element_type == "service":
            return rosservice.get_service_list()
        elif self._element_type == "param":
            return rospy.get_param_names()
        else:
            rospy.logerr("Requested ros element %s does not exist", self._element_type)
            return None


def wait_for_conditions(conditions_to_satisfy, timeout):
    time = rospy.Time.now() + rospy.Duration(timeout)
    all_conditions_satisfied = False

    while not all_conditions_satisfied:
        all_conditions_satisfied_list = []
        for condition in conditions_to_satisfy.values():
            all_conditions_satisfied_list.append(condition.check_if_required_element_is_available())
        if all(satisfied for satisfied in all_conditions_satisfied_list):
            all_conditions_satisfied = True
        if (round(rospy.Time.now().to_sec(), 1) == round(time.to_sec(), 1)):
            rospy.logerr("Timeout of {}s exceeded".format(timeout))
            for condition_type, condition in conditions_to_satisfy.iteritems():
                if condition.missing_elements:
                    rospy.logerr('Could not find the following {}s: {}'.format(condition_type,
                                                                               condition.missing_elements))
            break
    return all_conditions_satisfied


if __name__ == "__main__":
    rospy.init_node('conditional_delayed_roslaunch', anonymous=True)

    conditions_to_satisfy = {}

    package_name = rospy.get_param("~package_name")
    executable_name = rospy.get_param("~executable_name")
    arguments_list = rospy.get_param("~launch_args_list")
    timeout = rospy.get_param("~timeout")

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

    if all_conditions_satisfied:
        if executable_name.endswith('.launch'):
            os.system("roslaunch {} {} {}".format(package_name, executable_name, arguments_list))
        else:
            os.system("rosrun {} {} {}".format(package_name, executable_name, arguments_list))
    else:
        rospy.logfatal("{}: Could not launch {} {}, make sure all required conditions are met".format(rospy.get_name(),
                                                                                                      package_name,
                                                                                                      executable_name))
        rospy.signal_shutdown("Required components missing")
        sys.exit(1)
