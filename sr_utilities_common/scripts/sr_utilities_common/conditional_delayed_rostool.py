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
    def __init__(self, items_not_found):
        self.items_not_found = items_not_found

    def __str__(self):
        return "TimeoutException {}".format(self.items_not_found)

class RosConceptHandler(object):
    def __init__(self, concept_type):
        self._concept_type = concept_type
        self._check_rate = rospy.Rate(10)
        self._available_concepts = []
        self._not_available_concepts = []

    def wait_for_concept(self, required_concepts_array, timeout=None):
        all_concepts_available = False
        time = rospy.Time.now() + timeout
        while not all_concepts_available:
            roscore_published_concepts = self._retrieve_available_concept(self._concept_type)
            print("Av conc: ", roscore_published_concepts)
            self._check_available_concepts(roscore_published_concepts, required_concepts_array)
            if len(self._available_concepts) == len(required_concepts_array):
                all_concepts_available = True
            if (round(rospy.Time.now().to_sec(), 1) == round(time.to_sec(), 1)):
                self._get_not_available_concepts(required_concepts_array)
                raise TimeoutException(self._not_available_concepts)
            self._check_rate.sleep()
        return all_concepts_available

    def _retrieve_available_concept(self, object_type):
        if object_type == "topic":
            return rospy.get_published_topics()
        elif object_type == "service":
            return rosservice.get_service_list()
        elif object_type == "param":
            return rospy.get_param_names()

    def _check_available_concepts(self, roscore_published_concepts, required_concepts_array):
        for concept in required_concepts_array:
            if concept not in self._available_concepts:
                if any(concept in sublist for sublist in roscore_published_concepts):
                    rospy.loginfo("Found %s", concept)
                    self._available_concepts.append(concept)

    def _get_not_available_concepts(self, required_topics_array):
        for concept in required_topics_array:
            if concept not in self._available_concepts:
                self._not_available_concepts.append(concept)

class ConditionalDelayedRosTool(object):
    def __init__(self, package_name, executable_name, topics_array, params_array, services_array):
        self._package_name = package_name
        self._executable_name = executable_name
        self._topics_array = topics_array
        self._topic_handler = RosConceptHandler("topic")
        self._params_array = params_array
        self._param_handler = RosConceptHandler("param")
        self._services_array = services_array
        self._service_handler = RosConceptHandler("service")
        self._timeout = rospy.Duration(1.0)

    def _wait_for_conditions(self):
        """
        check result of the handlers
        """
        all_conditions_met = False

        try:
            wait_for_topic_process = Process(target=self._topic_handler.wait_for_concept(self._topics_array, self._timeout))
            wait_for_topic_process.start()
            wait_for_param_process = Process(target=self._param_handler.wait_for_concept(self._params_array, self._timeout))
            wait_for_param_process.start()
            wait_for_service_process = Process(target=self._service_handler.wait_for_concept(self._services_array, self._timeout))
            wait_for_service_process.start()
            wait_for_topic_process.join()
            wait_for_param_process.join()
            wait_for_service_process.join()
        except TimeoutException as error:
            rospy.logerr('Timeout error, could not find the following: %s', error.items_not_found)
        else:
            all_conditions_met = True

        return all_conditions_met

    def execute(self):
        """
        run if everything has been received
        """
        if self._wait_for_conditions():
            rospy.loginfo("launching launchfile")
        else:
            rospy.logwarn("could not launch sorry :(")

if __name__ == "__main__":
    rospy.init_node('conditional_delayed_roslaunch', anonymous=True)
    cdlrt = ConditionalDelayedRosTool("cacca", "pupu", ["/rosout"], ["/rosdistro", "pu"], ["/rosout/get_loggers", "gesu"])
    cdlrt.execute()
