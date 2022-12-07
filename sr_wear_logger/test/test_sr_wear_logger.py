#!/usr/bin/python3

# Copyright 2021-2022 Shadow Robot Company Ltd.
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
from unittest import TestCase
import rospy
import rospkg
import yaml
import rostest
from sr_wear_logger.sr_wear_logger import SrWearLogger


class TestSrWearLogger(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path_to_test_folder = rospkg.RosPack().get_path('sr_wear_logger') + "/test"
        cls.path_to_test_file = cls.path_to_test_folder + "/wear_data.yaml"
        cls.log_file_key_list = ["total_time_[s]", "total_angles_[rad]"]

    @classmethod
    def tearDownClass(cls):
        rospy.loginfo("Cleaning up")
        for test_file in os.listdir(cls.path_to_test_folder):
            pattern = r"\d\d\d\d-\d\d-\d\d-\d\d-\d\d-\d\d.yaml"
            if bool(re.match(pattern, test_file)):
                os.remove(cls.path_to_test_folder + "/" + test_file)
        if os.path.exists(cls.path_to_test_folder+"/wear_data.yaml"):
            os.remove(cls.path_to_test_folder + "/wear_data.yaml")

    @classmethod
    def check_are_values_numeric(cls, dictionary):
        for key, value in list(dictionary.items()):
            if isinstance(value, dict):
                dictionary[key] = cls.check_are_values_numeric(value)
            elif not(isinstance(dictionary[key], float) or isinstance(dictionary[key]), int):
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
                with open(self.path_to_test_file, 'r', encoding='UTF-8') as test_file:
                    data = yaml.load(test_file, Loader=yaml.SafeLoader)
                    success = all(key in list(data.keys()) for key in self.log_file_key_list)
                rospy.sleep(1)
            except Exception:
                pass
        self.assertTrue(success)

    def test_15_check_log_file_values(self):
        success = False
        if os.path.exists(self.path_to_test_file):
            try:
                with open(self.path_to_test_file, 'r', encoding='UTF-8') as test_file:
                    data = yaml.load(test_file, Loader=yaml.SafeLoader)
                    success = self.check_are_values_numeric(data)
                rospy.sleep(1)
            except Exception:
                pass
        self.assertTrue(success)


if __name__ == '__main__':
    PKGNAME = 'sr_wear_logger'
    NODENAME = 'test_sr_wear_logger'
    rospy.init_node(NODENAME, anonymous=True)

    rostest.rosrun(PKGNAME, NODENAME, TestSrWearLogger)
