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
from multiprocessing import Process

class TimeoutException(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "TimeoutException"

class RosElementsHandler(object):
    def __init__(self, element_type, required_elements_array):
        self._element_type = element_type
        self._required_elements_array = required_elements_array
        self.missing_elements = []

    def execute(self):
        all_elements_available = False
        roscore_published_elements = self._retrieve_available_elements(self._element_type)
        self._check_if_element_is_available(roscore_published_elements, self._required_elements_array)
        if 0 == len(self._required_elements_array):
            all_elements_available = True
        self.missing_elements = self._required_elements_array
        return all_elements_available

    def _retrieve_available_elements(self, object_type):
        if object_type == "topic":
            return rospy.get_published_topics()
        elif object_type == "service":
            return rosservice.get_service_list()
        elif object_type == "param":
            return rospy.get_param_names()

    def _check_if_element_is_available(self, roscore_published_elements, required_elements_array):
        for element in required_elements_array:
            if any(element in sublist for sublist in roscore_published_elements):
                rospy.loginfo("Found %s", element)
                required_elements_array.remove(element)


if __name__ == "__main__":
    rospy.init_node('conditional_delayed_roslaunch', anonymous=True)

    package_name = rospy.get_param("~package_name")
    executable_name = rospy.get_param("~executable_name")
    executable_type = rospy.get_param("~executable_type")
    topics_array = rospy.get_param("~topics_array")
    params_array = rospy.get_param("~params_array")
    services_array = rospy.get_param("~services_array")
    arguments_array = rospy.get_param("~arguments_array")
    timeout = rospy.get_param("~timeout")
    timeout = rospy.Duration(1.0)

    topic_handler = RosElementsHandler("topic", topics_array)
    param_handler = RosElementsHandler("param", params_array)
    service_handler = RosElementsHandler("service", services_array)

    time = rospy.Time.now() + timeout
    all_conditions_satisfied = False
    while not all_conditions_satisfied:
        all_topics_available = topic_handler.execute()
        all_params_available = param_handler.execute()
        all_services_available = service_handler.execute()
        if all_topics_available and all_params_available and all_services_available:
            all_conditions_satisfied = True
        if (round(rospy.Time.now().to_sec(), 1) == round(time.to_sec(), 1)):
            try:
                raise TimeoutException
            except TimeoutException as e:
                rospy.logerr("Timeout exceeded %s", e)
                if topic_handler.missing_elements:
                    rospy.logerr('Could not find the following topics: %s', topic_handler.missing_elements)
                if param_handler.missing_elements:
                    rospy.logerr('Could not find the following params: %s', param_handler.missing_elements)
                if service_handler.missing_elements:
                    rospy.logerr('Could not find the following sevices: %s', service_handler.missing_elements)
            break

    if all_conditions_satisfied:
        if executable_type == "node":
            os.system("rosrun {} {}")
        elif executable_type == "launch":
            print("roslaunch {} {} {}".format(package_name, executable_name, " ".join(arguments_array)))
            os.system("roslaunch {} {} {}".format(package_name, executable_name, " ".join(arguments_array)))
    else:
        print("Could not launch bro")
