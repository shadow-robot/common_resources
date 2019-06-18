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

import sys
import rospy
import psutil
from threading import Thread, Lock
from sr_watchdog.msg import TestStatus, SystemStatus


# TODO: Change to decided convention of enum and put inside of class
class Status:
    PENDING, OK, ERROR = (-1, 0, 1)


class SrWatchdog(object):
    def __init__(self, tested_system_name="tested system", checks_class=None,
                 error_checks_list=[], warning_checks_list=[]):
        self.tested_system_name = tested_system_name
        self.checks_class = checks_class
        self.error_checks_list = error_checks_list
        self.warning_checks_list = warning_checks_list

        self.watchdog_publisher = rospy.Publisher('sr_watchdog', SystemStatus, queue_size=10)
        self.demo_status = Status.PENDING
        self.node_logs = []
        self.check_results = []
        self.checks_done_in_current_cycle = 0

    def main_thread_method(self):
        while not rospy.is_shutdown():
            self.report_status()

    def checks_thread_method(self):
        while not rospy.is_shutdown():
            self.run_checks()

    def run(self):
        main_thread = Thread(target=self.main_thread_method).start()
        checks_thread = Thread(target=self.checks_thread_method).start()

    def report_status(self):
        rate = rospy.Rate(1)
        system_status = SystemStatus()
        system_status.system_name = self.tested_system_name
        system_status.checks_cycle_completion = self.checks_done_in_current_cycle / float(len(self.error_checks_list) +
                                                                                          len(self.warning_checks_list)) * 100
        system_status.status = self.demo_status

        system_status.test_statuses = self.check_results
        system_status.logs = [log[0] for log in self.node_logs]
        self.watchdog_publisher.publish(system_status)
        rate.sleep()

    def run_checks(self):
        self.checks_done_in_current_cycle = 0
        self.run_error_checks()
        self.run_warning_checks()
        rospy.sleep(1)

    def run_status_checks(self, check_type):
        if 'err' == check_type:
            checks_list = self.error_checks_list
            msg_type = 'ERROR'
        elif 'warn' == check_type:
            checks_list = self.warning_checks_list
            msg_type = 'WARN'
        else:
            raise ValueError("Wrong status check type")

        for check in checks_list:
            if check not in [check_result.test_name for check_result in self.check_results]:
                new_test = TestStatus()
                new_test.test_name = check
                new_test.test_type = msg_type
                new_test.result = True
                self.check_results.append(new_test)
            method_to_call = getattr(self.checks_class, check)
            try:
                return_value = method_to_call()
            except Exception as ex:
                self.node_logs.append(("[WARN] Check \'{}\' threw an exception: \'{}\'. Skipping..."
                                       .format(check, type(ex).__name__), 'warn'))
                self.checks_done_in_current_cycle += 1
                continue

            if isinstance(return_value, bool):
                result = return_value
                error_msg = None
            elif isinstance(return_value, tuple) and 2 == len(return_value):
                result = return_value[0]
                error_msg = return_value[1]
            else:
                self.node_logs.append(("[WARN] Wrong method result format for \'{}\'. "
                                       "Need either a bool or (bool, string) tuple!".format(check), 'warn'))
                continue

            if result != [check_result.result for check_result in self.check_results if check == check_result.test_name]:
                if not result:
                    if error_msg is None:
                        error_log = ("[{}] Check \'{}\' failed!".format(msg_type, check), check_type)
                    else:
                        error_log = ("[{}] Check \'{}\' failed with message: {}"
                                     .format(msg_type, check, error_msg), check_type)
                    self.node_logs.append(error_log)
                else:
                    self.node_logs.append(("[INFO] Check \'{}\' passing now!".format(check), 'info'))

            for i in range(len(self.check_results)):
                if check == self.check_results[i].test_name:
                    self.check_results[i].result = result

            if not result and 'ERROR' == msg_type:
                self.demo_status = Status.ERROR

            self.checks_done_in_current_cycle += 1

        if False not in [check_result.result for check_result in self.check_results if 'ERROR' == check_result.test_type]:
            self.demo_status = Status.OK

    def run_error_checks(self):
        self.run_status_checks('err')

    def run_warning_checks(self):
        self.run_status_checks('warn')


class TestChecksClass(object):
    def __init__(self):
        self.tmp = [0, 0, 0]

    def mock_check_robot_clear_from_collision(self):
        rospy.sleep(2)
        if self.tmp[0] != 1:
            self.tmp[0] = 1
            return False
        else:
            self.tmp[0] = 0
            return True

    def mock_check_if_arm_running(self):
        rospy.sleep(3)
        if self.tmp[1] != 0:
            self.tmp[1] = 0
            return False
        else:
            self.tmp[1] = 1
            return True

    def mock_check_if_hand_running(self):
        rospy.sleep(1)
        self.tmp[2] += 1
        if self.tmp[2] > 5 and self.tmp[2] < 10:
            return (False, "Hand not running!")
        else:
            return True


if __name__ == '__main__':
    rospy.init_node('mock_sr_teleop_watchdog')

    test_class = TestChecksClass()
    error_checks_list = ['mock_check_if_arm_running', 'mock_check_if_hand_running']
    warning_checks_list = ['mock_check_robot_clear_from_collision']

    mock_watchdog = SrWatchdog("mock system", test_class, error_checks_list, warning_checks_list)
    mock_watchdog.run()
    rospy.spin()
