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
from sr_watchdog.watchdog import SrWatchdog

class TestChecksClass(object):
    def __init__(self):
        self.tmp = [0, 0, 0]

    def mock_check_robot_clear_from_collision(self):
        rospy.sleep(6)
        if self.tmp[0] != 1:
            self.tmp[0] = 1
            return False
        else:
            self.tmp[0] = 0
            return True

    def mock_check_if_arm_running(self):
        rospy.sleep(10)
        if self.tmp[1] != 0:
            self.tmp[1] = 0
            return False
        else:
            self.tmp[1] = 1
            return True

    def mock_check_if_hand_running(self):
        rospy.sleep(3)
        self.tmp[2] += 1
        if self.tmp[2] > 5 and self.tmp[2] < 10:
            return (False, "Hand not running!")
        elif 36 == self.tmp[2]:
            self.tmp[2] = 0
            return True
        else:
            return True

    def mock_check_returning_wrong_format(self):
        rospy.sleep(2)
        return None

    def mock_check_throwing_exception(self):
        rospy.sleep(2)
        raise ValueError


if __name__ == '__main__':
    rospy.init_node('mock_sr_teleop_watchdog')

    test_class = TestChecksClass()
    error_checks_list = ['mock_check_if_arm_running', 'mock_check_if_hand_running', 'mock_check_returning_wrong_format']
    warning_checks_list = ['mock_check_robot_clear_from_collision', 'mock_check_throwing_exception']

    mock_watchdog = SrWatchdog("mock system", test_class, error_checks_list, warning_checks_list)
    mock_watchdog.run()
    rospy.spin()
