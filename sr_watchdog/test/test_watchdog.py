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
from sr_watchdog.watchdog import SrWatchdog, SrWatchdogChecks
from std_msgs.msg import Bool


class TestChecksClass(SrWatchdogChecks):
    def __init__(self):
        SrWatchdogChecks.__init__(self, "default")
        self.error_check_pass_flag = True
        self.warn_check_pass_flag = True
        rospy.Subscriber("/watchdog_test/set_error_check_result", Bool, self.set_error_check)
        rospy.Subscriber("/watchdog_test/set_warning_check_result", Bool, self.set_warn_check)

    def set_warn_check(self, result):
        self.warn_check_pass_flag = result.data

    def set_error_check(self, result):
        self.error_check_pass_flag = result.data

    @SrWatchdogChecks.watchdog_warning_check
    def test_check_warn_type(self):
        rospy.sleep(2)
        return self.warn_check_pass_flag

    @SrWatchdogChecks.watchdog_error_check
    def test_check_error_type(self):
        rospy.sleep(2)
        if not self.error_check_pass_flag:
            return (False, "Test message!")
        else:
            return True

    @SrWatchdogChecks.watchdog_error_check
    def test_check_wrong_return_format(self):
        rospy.sleep(2)
        return None

    @SrWatchdogChecks.watchdog_error_check
    def test_check_throwing_exception(self):
        rospy.sleep(2)
        raise ValueError


if __name__ == '__main__':
    rospy.init_node('watchdog_test')
    rospy.loginfo("Test watchdog started...")

    test_class = TestChecksClass()
    mock_watchdog = SrWatchdog("mock system", [test_class])
    mock_watchdog.run()
    rospy.spin()
