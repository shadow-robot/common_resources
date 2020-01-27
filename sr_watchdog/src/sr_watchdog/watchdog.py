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
import inspect
from threading import Thread, Lock
from optparse import OptionParser
from sr_watchdog.msg import CheckStatus, SystemStatus, SystemLog


class SrWatchdogExceptions(Exception):
    pass


class CheckThrowingException(SrWatchdogExceptions):
    pass


class CheckResultWrongFormat(SrWatchdogExceptions):
    pass


class SrWatchdog(object):
    def __init__(self, tested_system_name="tested system", checks_classes_list=[]):
        self.tested_system_name = tested_system_name
        self.checks_classes_list = checks_classes_list
        self.logs_remembered = rospy.get_param('~logs_remembered', 10)

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

        if self.logs_remembered < len(self.watchdog_logs):
            del self.watchdog_logs[0]

        self.watchdog_publisher.publish(system_status)
        rate.sleep()

    def _create_new_check_object(self, check_name, component, check_type, check_class_name):
        new_test = CheckStatus()
        new_test.check_name = check_name
        new_test.component = component
        new_test.check_type = check_type
        new_test.result = True
        new_test.check_class_name = check_class_name
        return new_test

    def _parse_checks(self):
        for checks_class in self.checks_classes_list:
            component = checks_class.component
            for watchdog_check_name in checks_class.get_all_watchdog_check_names():
                check_type = getattr(checks_class, watchdog_check_name).__dict__['check_type']
                self.checks_list.append(self._create_new_check_object(watchdog_check_name,
                                                                      component,
                                                                      check_type,
                                                                      checks_class.__class__.__name__))

    def _find_class_corresponding_to_check(self, check):
        for checks_class in self.checks_classes_list:
            if checks_class.__class__.__name__ == check.check_class_name:
                return checks_class

    def _run_single_check(self, check):
        used_class = self._find_class_corresponding_to_check(check)
        method_to_call = getattr(used_class, check.check_name)
        try:
            result = method_to_call()
        except CheckResultWrongFormat:
            self.watchdog_logs.append(("[WARN] Wrong method result format for \'{}\'. "
                                       "Need either a bool or (bool, string) tuple!"
                                       " Skipping and blacklisting this check..."
                                       .format(check.check_name), SystemLog.WARN))
            raise CheckResultWrongFormat
        except Exception as ex:
            self.watchdog_logs.append(("[WARN] Check \'{}\' threw an exception: \'{}: {}\'."
                                       " Skipping and blacklisting this check..."
                                       .format(check.check_name, type(ex).__name__, str(ex)), SystemLog.WARN))
            raise CheckThrowingException
        return result

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
        if False not in [check_result.result for check_result in self.checks_list
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
                                 .format("WARN" if SystemLog.WARN == log_type else "ERROR", check.check_name),
                                 log_type)
                else:
                    error_log = ("[{}] Check \'{}\' failed with message: {}"
                                 .format("WARN" if SystemLog.WARN == log_type else "ERROR", check.check_name,
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
                check_return_value = self._run_single_check(check)
            except (CheckThrowingException, CheckResultWrongFormat):
                self.checks_done_in_current_cycle += 1
                checks_blacklist.append(check.check_name)
                continue

            if check_return_value['result'] != check.result:
                self._add_system_log_on_check_result_change(check, check_return_value['result'],
                                                            check_return_value['error_msg'])
                self._update_check_result(check.check_name, check_return_value['result'])

            self._refresh_system_status()

            self.checks_done_in_current_cycle += 1

        for check_name in checks_blacklist:
            self.checks_done_in_current_cycle -= 1
            self._blacklist_single_check(check_name)
        rospy.sleep(1)


class SrWatchdogChecks(object):
    def __init__(self, component=None):
        self.component = component

    def _get_all_class_method_names(self):
        return [member[0] for member in inspect.getmembers(self, predicate=inspect.ismethod)]

    def _check_if_method_is_a_watchdog_check(self, method_name):
        method = getattr(self, method_name)
        metadata = method.__dict__
        if 'decorated' in metadata:
            if metadata['decorated']:
                return True
        return False

    def get_all_watchdog_check_names(self):
        watchdog_check_names = []
        all_method_names = self._get_all_class_method_names()
        for method_name in all_method_names:
            if self._check_if_method_is_a_watchdog_check(method_name):
                watchdog_check_names.append(method_name)
        return watchdog_check_names

    @staticmethod
    def watchdog_error_check(function):
        wrapper = SrWatchdogChecks.watchdog_check_decorator(function)
        wrapper.check_type = CheckStatus.ERROR
        return wrapper

    @staticmethod
    def watchdog_warning_check(function):
        wrapper = SrWatchdogChecks.watchdog_check_decorator(function)
        wrapper.check_type = CheckStatus.WARN
        return wrapper

    @staticmethod
    def watchdog_check_decorator(function):
        def wrapper(self):
            CONST_RETURN_TUPLE_EXPECTED_SIZE = 2
            return_value = function(self)
            if isinstance(return_value, bool):
                result = return_value
                error_msg = None
            elif isinstance(return_value, tuple) and \
                    CONST_RETURN_TUPLE_EXPECTED_SIZE == len(return_value):
                result = return_value[0]
                error_msg = return_value[1]
            else:
                raise CheckResultWrongFormat
            return {"result": result, "error_msg": error_msg}
        wrapper.decorated = True
        return wrapper
