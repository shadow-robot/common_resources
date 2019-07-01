#!/usr/bin/python

# Copyright 2019 Shadow Robot Company Ltd.
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
from std_msgs.msg import Bool
from sr_utilities_common.manual_test_suite import ManualTestSuite


class TestWatchdogClient(object):
    def __init__(self):
        self.check_warn_publisher = rospy.Publisher('/watchdog_test/set_warning_check_result',
                                                    Bool, queue_size=10)
        self.check_error_publisher = rospy.Publisher('/watchdog_test/set_error_check_result',
                                                     Bool, queue_size=10)

    def fail_test_check_warn_type(self):
        bool_false = Bool()
        bool_false.data = False
        self.check_warn_publisher.publish(bool_false)
        return True

    def fail_test_check_error_type(self):
        bool_false = Bool()
        bool_false.data = False
        self.check_error_publisher.publish(bool_false)
        return True

    def pass_test_check_warn_type(self):
        bool_true = Bool()
        bool_true.data = True
        self.check_warn_publisher.publish(bool_true)
        return True

    def pass_test_check_error_type(self):
        bool_true = Bool()
        bool_true.data = True
        self.check_error_publisher.publish(bool_true)
        return True

if __name__ == '__main__':
    rospy.init_node('watchdog_test_client')

    test_watchdog_client = TestWatchdogClient()
    ordered_test_method_list = ['fail_test_check_warn_type', 'pass_test_check_warn_type',
                                'fail_test_check_error_type', 'pass_test_check_error_type']
    test_suite = ManualTestSuite(test_watchdog_client, ordered_test_method_list)
