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


class TestSrWearLogger(TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        path_to_test_file = rospkg.RosPack().get_path('sr_wear_logger') + "/test/wear_data.yaml"
        if os.path.exists(path_to_test_file):
            os.remove(path_to_test_file)

    def test_10_check_param_wrong_hand_serial(self):
        test_wear_logger = SrWearLogger("", 1, 1)
        self.assertFalse(test_wear_logger.check_parameters())

    def test_11_check_param_wrong_hand_time(self):
        test_wear_logger = SrWearLogger("test", 1.2, 1)
        self.assertFalse(test_wear_logger.check_parameters())

    def test_12_check_param_wrong_hand_time(self):
        test_wear_logger = SrWearLogger("test", 1.2, 1)
        self.assertFalse(test_wear_logger.check_parameters())

    def test_13_check_param_all_correct(self):
        test_wear_logger = SrWearLogger("test", 10, 2)
        self.assertTrue(test_wear_logger.check_parameters())

    def test_14_check_if_log_file_was_created(self):
        test_wear_logger = SrWearLogger("test", 10, 1)
        test_wear_logger.run()
        path_to_test_file = rospkg.RosPack().get_path('sr_wear_logger') + "/test/wear_data.yaml"
        self.assertTrue(os.path.exists(path_to_test_file))


if __name__ == '__main__':
    PKGNAME = 'sr_wear_logger'
    NODENAME = 'test_sr_wear_logger'
    rospy.init_node(NODENAME, anonymous=True)

    rostest.rosrun(PKGNAME, NODENAME, TestSrWearLogger)
