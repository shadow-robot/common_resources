#!/usr/bin/python

# Copyright 2021 Shadow Robot Company Ltd.
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
import rospkg
import rostest
from unittest import TestCase
from std_msgs.msg import Bool
from sr_utilities_common.manual_test_suite import ManualTestSuite
from sr_utilities_common.aws_manager import AWS_Manager
from sr_wear_logger.sr_wear_logger import SrWearLogger
from sr_wear_logger.dummy_publisher import DummyPublisher
import os
import re
import yaml


class TestSrWearLogger(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path_to_test_folder = rospkg.RosPack().get_path('sr_wear_logger') + "/test"
        cls.path_to_test_file = cls.path_to_test_folder + "/wear_data.yaml"
        cls.log_file_key_list = ["total_time_[s]", "total_angles_[rad]"]

    @classmethod
    def tearDownClass(cls):
        rospy.loginfo("Cleaning up")
        for file in os.listdir(cls.path_to_test_folder):
            pattern = "\d\d\d\d-\d\d-\d\d-\d\d-\d\d-\d\d.yaml"
            if bool(re.match(pattern, file)):
                os.remove(cls.path_to_test_folder + "/" + file)
        if os.path.exists(cls.path_to_test_folder+"/wear_data.yaml"):
            os.remove(cls.path_to_test_folder + "/wear_data.yaml")

    @classmethod
    def check_are_values_numeric(self, dictionary):
        for k, v in dictionary.items():
            if isinstance(v, dict):
                dictionary[k] = self.check_are_values_numeric(v)
            else:
                if not(type(dictionary[k]) is float or type(dictionary[k]) is int):
                    return False
        return True

    def test_10_check_param_case_wrong_hand_serial(self):
        test_wear_logger = SrWearLogger("", "rh", 1, 1)
        self.assertFalse(test_wear_logger.check_parameters())

    def test_11_check_param_case_wrong_time(self):
        test_wear_logger = SrWearLogger("test", "rh", 1.2, 1)
        self.assertFalse(test_wear_logger.check_parameters())

    def test_12_check_param_case_all_correct(self):
        test_wear_logger = SrWearLogger("test", "rh", 3, 1)
        self.assertTrue(test_wear_logger.check_parameters())

    def test_13_check_log_file_existence(self):
        test_wear_logger = SrWearLogger("test", "rh", 3, 1)
        test_wear_logger.run()
        rospy.sleep(5)
        test_wear_logger.stop()
        self.assertTrue(os.path.exists(self.path_to_test_file))

    def test_14_check_log_file_structure(self):
        success = False
        if os.path.exists(self.path_to_test_file):
            try:
                f = open(self.path_to_test_file, 'r')
                data = yaml.load(f, Loader=yaml.SafeLoader)
                success = all(k in data.keys() for k in self.log_file_key_list)
                f.close()
                rospy.sleep(1)
            except Exception as e:
                pass
        self.assertTrue(success)

    def test_15_check_log_file_values(self):
        success = False
        if os.path.exists(self.path_to_test_file):
            try:
                f = open(self.path_to_test_file, 'r')
                data = yaml.load(f, Loader=yaml.SafeLoader)
                success = self.check_are_values_numeric(data)
                f.close()
                rospy.sleep(1)
            except Exception as e:
                pass
        self.assertTrue(success)


if __name__ == '__main__':
    PKGNAME = 'sr_wear_logger'
    NODENAME = 'test_sr_wear_logger'
    rospy.init_node(NODENAME, anonymous=True)

    rostest.rosrun(PKGNAME, NODENAME, TestSrWearLogger)
