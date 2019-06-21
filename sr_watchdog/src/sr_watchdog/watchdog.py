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
from sr_watchdog.msg import CheckStatus, SystemStatus, SystemLog


class SrWatchdogExceptions(Exception):
    pass


class CheckThrowingException(SrWatchdogExceptions):
    pass


class CheckResultWrongFormat(SrWatchdogExceptions):
    pass


class SrWatchdog(object):
    def __init__(self, tested_system_name="tested system", checks_class=None,
                 error_check_names_list=[], warning_check_names_list=[]):
        self.tested_system_name = tested_system_name
        self.checks_class = checks_class
        self.error_check_names_list = error_check_names_list
        self.warning_check_names_list = warning_check_names_list

        self.watchdog_publisher = rospy.Publisher('sr_watchdog', SystemStatus, queue_size=10)
        self.demo_status = SystemStatus.PENDING
        self.watchdog_logs = []
        self.checks_list = []
        self.checks_done_in_current_cycle = 0

        self._parse_checks()

    def run(self):
        main_thread = Thread(target=self._report_thread_method).start()
        checks_thread = Thread(target=self._checks_thread_method).start()

    def _report_thread_method(self):
        while not rospy.is_shutdown():
            self._report_status()

    def _checks_thread_method(self):
        while not rospy.is_shutdown():
            self._run_all_checks()

    def _report_status(self):
        rate = rospy.Rate(10)
        system_status = SystemStatus()
        system_status.system_name = self.tested_system_name
        system_status.checks_cycle_completion = int(round(self.checks_done_in_current_cycle /
                                                          float(len(self.checks_list)) * 100))
        system_status.status = self.demo_status
        system_status.check_statuses = self.checks_list

        for log in self.watchdog_logs:
            new_log = SystemLog()
            new_log.msg = log[0]
            new_log.type = log[1]
            system_status.logs.append(new_log)

        if 10 < len(self.watchdog_logs):
            del self.watchdog_logs[0]

        self.watchdog_publisher.publish(system_status)
        rate.sleep()

    def _create_new_check_object(self, check_name, check_type, initial_check_result=True):
        new_test = CheckStatus()
        new_test.check_name = check_name
        new_test.check_type = check_type
        new_test.result = True
        return new_test

    def _parse_checks(self):
        for error_check_name in self.error_check_names_list:
            self.checks_list.append(self._create_new_check_object(error_check_name, CheckStatus.ERROR))
        for warn_check_name in self.warning_check_names_list:
            self.checks_list.append(self._create_new_check_object(warn_check_name, CheckStatus.WARN))

    def _run_single_check(self, check_name):
        method_to_call = getattr(self.checks_class, check_name)
        try:
            return_value = method_to_call()
        except Exception as ex:
            self.watchdog_logs.append(("[WARN] Check \'{}\' threw an exception: \'{}\'."
                                       " Skipping and blacklisting this check..."
                                       .format(check_name, type(ex).__name__), SystemLog.WARN))
            raise CheckThrowingException

        if isinstance(return_value, bool):
            result = return_value
            error_msg = None
        elif isinstance(return_value, tuple) and 2 == len(return_value):
            result = return_value[0]
            error_msg = return_value[1]
        else:
            self.watchdog_logs.append(("[WARN] Wrong method result format for \'{}\'. "
                                       "Need either a bool or (bool, string) tuple!"
                                       " Skipping and blacklisting this check..."
                                       .format(check_name), SystemLog.WARN))
            raise CheckResultWrongFormat
        return (result, error_msg)

    def _update_check_result(self, check_name, new_result):
        for i in range(len(self.checks_list)):
            if check_name == self.checks_list[i].check_name:
                self.checks_list[i].result = new_result
                return

    def _map_check_type_to_log_type(self, check_type):
        if CheckStatus.ERROR == check_type:
            log_type = SystemLog.ERROR
        elif CheckStatus.WARN == check_type:
            log_type = SystemLog.WARN
        else:
            raise ValueError("Wrong status check type")
        return log_type

    def _blacklist_single_check(self, check_name):
        for i in range(len(self.checks_list)):
            if check_name == self.checks_list[i].check_name:
                del self.checks_list[i]
                return

    def _refresh_system_status(self):
        if False not in [check_result.result for check_result in self.checks_list \
                         if CheckStatus.ERROR == check_result.check_type]:
            self.demo_status = SystemStatus.OK
        else:
            self.demo_status = SystemStatus.ERROR

    def _add_system_log_on_check_result_change(self, check, new_result, error_msg):
        log_type = self._map_check_type_to_log_type(check.check_type)
        if new_result != check.result:
            if not new_result:
                if error_msg is None:
                    error_log = ("[{}] Check \'{}\' failed!"
                                 .format("WARN" if SystemLog.WARN == log_type else "ERROR",
                                 check.check_name), log_type)
                else:
                    error_log = ("[{}] Check \'{}\' failed with message: {}"
                                 .format("WARN" if SystemLog.WARN == log_type else "ERROR",
                                 check.check_name,
                                 error_msg), log_type)
                self.watchdog_logs.append(error_log)
            else:
                self.watchdog_logs.append(("[INFO] Check \'{}\' passing now!".format(check.check_name), SystemLog.INFO))
        else:
            raise ValueError("Check result has not changed!")

    def _run_all_checks(self):
        checks_blacklist = []
        self.checks_done_in_current_cycle = 0
        for check in self.checks_list:
            try:
                result, error_msg = self._run_single_check(check.check_name)
            except (CheckThrowingException, CheckResultWrongFormat):
                self.checks_done_in_current_cycle += 1
                checks_blacklist.append(check.check_name)
                continue

            if result != check.result:
                self._add_system_log_on_check_result_change(check, result, error_msg)
                self._update_check_result(check.check_name, result)

            self._refresh_system_status()

            self.checks_done_in_current_cycle += 1

        for check_name in checks_blacklist:
            self.checks_done_in_current_cycle -= 1
            self._blacklist_single_check(check_name)

        rospy.sleep(1)
