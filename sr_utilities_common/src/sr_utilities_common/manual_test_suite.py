#!/usr/bin/python3

# Copyright 2019, 2022 Shadow Robot Company Ltd.
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

from builtins import input
import argparse
import rospy


class ManualTestSuite:
    def __init__(self, test_case_class, ordered_test_method_list, unattended=False):
        self.test_case_class = test_case_class
        self.ordered_test_method_list = ordered_test_method_list
        self.unattended = unattended
        self.test_results = {'total': 0, 'passed': 0, 'failed': 0}
        self.color_codes = {'green': '\033[92m', 'red': '\033[91m',
                            'orange': '\033[93m', 'default': '\033[0m'}
        self._create_all_tests()
        self._print_summary()

    def _create_all_tests(self):
        for test_method_name in self.ordered_test_method_list:
            test_method = getattr(self.test_case_class, test_method_name)
            self._create_test(test_method)

    def _create_test(self, test_method):
        if not self.unattended:
            input(self.color_codes['orange'] +
                  'Test {}: {}. Press [RETURN] to continue...'.format(self.test_results['total'],
                                                                      test_method.__name__))
        self.test_results['total'] += 1
        result = test_method()
        if result:
            rospy.logwarn('{} test passed'.format(test_method.__name__))
            self.test_results['passed'] += 1
        else:
            rospy.logwarn('{} test failed!'.format(test_method.__name__))
            self.test_results['failed'] += 1

    def _print_summary(self):
        if self.test_results['failed'] > 0:
            result = 'FAILURE'
            result_color = self.color_codes['red']
        else:
            result = 'SUCCESS'
            result_color = self.color_codes['green']

        rospy.loginfo('\nSUMMARY\n' +
                      result_color + ' * RESULT: {}\n'.format(result) + self.color_codes['default'] +
                      ' * TESTS: {}\n'.format(self.test_results['total']) +
                      ' * FAILURES: {}'.format(self.test_results['failed']))

        if result == 'FAILURE':
            return False
        return True


class DummyClass:
    def __init__(self):
        self.dummy_variable = 1

    def dummy_method_1(self):
        return self.dummy_variable == 1

    def dummy_method_2(self):
        return 2 * self.dummy_variable


class DummyClassClient:
    def __init__(self):
        self.dummy_class_instance = DummyClass()

    def test_dummy_method_1(self):
        return self.dummy_class_instance.dummy_method_1()

    def test_dummy_method_2(self):
        return self.dummy_class_instance.dummy_method_2() == 2


if __name__ == '__main__':
    rospy.init_node('manual_test_suite')
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    dummy_class_client_arg = DummyClassClient()
    ordered_test_method_list_arg = ['test_dummy_method_2', 'test_dummy_method_1']
    test_suite = ManualTestSuite(dummy_class_client_arg, ordered_test_method_list_arg, unattended=args.unattended)
