#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import argparse
import rospy
import unittest
import rostest
import os
from std_msgs.msg import String
from sr_utilities_common.conditional_delayed_rostool import RosElementsHandler
from sr_utilities_common.conditional_delayed_rostool import wait_for_conditions

class ConditionalDelayedRosToolTestCase(unittest.TestCase):
    @classmethod
    def setUp(self):
        rospy.wait_for_message("test_mocap_topic", String)

    @classmethod
    def tearDown(self):
        pass

    def test_topic_element_type_is_valid(self):
        element_type = "topic"
        topics_list = ["test_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler(element_type, topics_list)
        result = ros_topic_handler_class._retrieve_available_elements("topic")
        self.assertIsNotNone(result)

    def test_param_element_type_is_valid(self):
        element_type = "param"
        params_list = ["test_mocap_param"]
        ros_param_handler_class = RosElementsHandler(element_type, params_list)
        result = ros_param_handler_class._retrieve_available_elements(element_type)
        self.assertIsNotNone(result)

    def test_service_element_type_is_valid(self):
        element_type = "service"
        services_list = ["test_mocap_topic"]
        ros_service_handler_class = RosElementsHandler(element_type, services_list)
        result = ros_service_handler_class._retrieve_available_elements(element_type)
        self.assertIsNotNone(result)

    def test_invalid_element_type(self):
        element_type = "invalid_element_type"
        services_list = ["test_mocap_topic"]
        self.ros_topic_handler_class = RosElementsHandler(element_type, services_list)
        result = self.ros_topic_handler_class._retrieve_available_elements(element_type)
        self.assertIsNone(result)

    def test_check_if_required_element_is_published_list(self):
        required_topics_list = ["/test_mocap_topic"]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_published()
        self.assertEquals(all_elements_available, True)

    def test_check_if_required_element_is_not_published_list(self):
        required_topics_list = ["non_published_test_topic"]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_published()
        self.assertEquals(all_elements_available, False)

    def test_check_if_element_is_available_empty_list(self):
        required_topics_list = []
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        all_elements_available = ros_topic_handler_class.check_if_required_element_is_published()
        self.assertEquals(all_elements_available, True)

    def test_check_if_elements_is_available_empty_string(self):
        required_topics_list=[""]
        ros_topic_handler_class = RosElementsHandler("topic", required_topics_list)
        try: 
            all_elements_available = ros_topic_handler_class.check_if_required_element_is_published()
        except ValueError as e: 
            self.assertEqual(type(e), ValueError)

    def test_requested_topic_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        topics_list = ["test_non_existing_topic"]
        conditions_to_satisfy["topic"] = RosElementsHandler("topic", topics_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, False)
    
    def test_requested_topic_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        topics_list = ["/test_mocap_topic"]
        conditions_to_satisfy["topic"] = RosElementsHandler("topic", topics_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, True)

    def test_requested_service_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        services_list = ["test_non_existing_service"]
        conditions_to_satisfy["service"] = RosElementsHandler("service", services_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, False)

    def test_requested_service_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        services_list = ["test_mocap_service"]
        conditions_to_satisfy["service"] = RosElementsHandler("service", services_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, True)

    def test_requested_param_not_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        params_list = ["test_non_existing_param"]
        conditions_to_satisfy["param"] = RosElementsHandler("param", params_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, False)

    def test_requested_param_available(self):
        conditions_to_satisfy = {}
        timeout = 0.1
        params_list = ["test_mocap_param"]
        conditions_to_satisfy["param"] = RosElementsHandler("param", params_list)
        all_conditions_satisfied = wait_for_conditions(conditions_to_satisfy, timeout)
        self.assertEquals(all_conditions_satisfied, True)

if __name__ == '__main__':
    PKGNAME = 'sr_utilities_common'
    NODENAME = 'conditional_delayed_roslaunch'
    rospy.init_node(NODENAME)

    rostest.rosrun(PKGNAME, NODENAME, ConditionalDelayedRosToolTestCase)

    os._exit(0)
