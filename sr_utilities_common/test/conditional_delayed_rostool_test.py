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
# pylint: disable=W0212

import sys
import unittest
import rospy
import rostest
from std_msgs.msg import String
from sr_utilities_common.conditional_delayed_rostool import RosElementsHandler
from sr_utilities_common.conditional_delayed_rostool import wait_for_conditions


class ConditionalDelayedRosToolTestCase(unittest.TestCase):
    @classmethod
    def setUp(cls):
        rospy.wait_for_message("test_mocap_topic", String)

    @classmethod
    def tearDown(cls):
        pass

    def test_topic_element_type_is_valid(self):
        element_type = "topic"
        topics_list = ["test_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler(element_type, topics_list)
        result = ros_topic_handler_class._retrieve_available_elements()  # pylint: disable=W0212
        self.assertIsNotNone(result)

    def test_param_element_type_is_valid(self):
        element_type = "param"
        params_list = ["test_mocap_param"]
        ros_param_handler_class = RosElementsHandler(element_type, params_list)
        result = ros_param_handler_class._retrieve_available_elements()
        self.assertIsNotNone(result)

    def test_service_element_type_is_valid(self):
        element_type = "service"
        services_list = ["test_mocap_topic"]
        ros_service_handler_class = RosElementsHandler(element_type, services_list)
        result = ros_service_handler_class._retrieve_available_elements()
        self.assertIsNotNone(result)

    def test_invalid_element_type(self):
        element_type = "invalid_element_type"
        services_list = ["test_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler(element_type, services_list)
        result = ros_topic_handler_class._retrieve_available_elements()
        self.assertIsNone(result)

    def test_check_if_required_element_is_available_list(self):
        required_topics_list = ["/test_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_available()
        self.assertTrue(all_elements_available)

    def test_check_by_regex_if_required_element_is_available_list(self):
        required_topics_list = ["/(test|mock)_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_available()
        self.assertTrue(all_elements_available)

    def test_check_if_required_element_is_not_available_list(self):
        required_topics_list = ["non_published_test_topic"]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_available()
        self.assertFalse(all_elements_available)

    def test_check_if_element_is_available_empty_list(self):
        required_topics_list = []
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_available()
        self.assertTrue(all_elements_available)

    def test_check_if_elements_is_available_empty_string(self):
        required_topics_list = [""]
        with self.assertRaises(ValueError):
            ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)  # pylint: disable=W0612

    def test_check_if_elements_is_available_not_a_string(self):
        required_topics_list = [0]
        with self.assertRaises(ValueError):
            ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)  # pylint: disable=W0612

    def test_requested_topic_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        topics_list = ["test_non_existing_topic"]
        conditions_to_satisfy["topic"] = RosElementsHandler("topic", topics_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertFalse(all_conditions_satisfied)

    def test_requested_topic_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        topics_list = ["/test_mocap_topic"]
        conditions_to_satisfy["topic"] = RosElementsHandler("topic", topics_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertTrue(all_conditions_satisfied)

    def test_requested_service_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        services_list = ["test_non_existing_service"]
        conditions_to_satisfy["service"] = RosElementsHandler("service", services_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertFalse(all_conditions_satisfied)

    def test_requested_service_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        services_list = ["/test_mocap_service"]
        conditions_to_satisfy["service"] = RosElementsHandler("service", services_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertTrue(all_conditions_satisfied)

    def test_requested_param_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        params_list = ["test_non_existing_param"]
        conditions_to_satisfy["param"] = RosElementsHandler("param", params_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertFalse(all_conditions_satisfied)

    def test_requested_param_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        params_list = ["/test_mocap_param"]
        conditions_to_satisfy["param"] = RosElementsHandler("param", params_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertTrue(all_conditions_satisfied)


if __name__ == '__main__':
    PKGNAME = 'sr_utilities_common'
    NODENAME = 'conditional_delayed_roslaunch'
    rospy.init_node(NODENAME)

    rostest.rosrun(PKGNAME, NODENAME, ConditionalDelayedRosToolTestCase)

    sys.exit(0)
