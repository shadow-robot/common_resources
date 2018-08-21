#!/usr/bin/python

import rospy
import sys
import argparse


class ManualTestSuite(object):
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
            raw_input(self.color_codes['orange'] +
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

        if 'FAILURE' == result:
            exit(1)
        exit(0)


class DummyClass(object):
    def __init__(self):
        self.dummy_variable = 1

    def dummy_method_1(self):
        if 1 == self.dummy_variable:
            return True

    def dummy_method_2(self):
            return 2 * self.dummy_variable


class DummyClassClient(object):
    def __init__(self):
        self.dummy_class_instance = DummyClass()

    def test_dummy_method_1(self):
        return self.dummy_class_instance.dummy_method_1()

    def test_dummy_method_2(self):
        if 2 == self.dummy_class_instance.dummy_method_2():
            return True
        return False

if __name__ == '__main__':
    rospy.init_node('manual_test_suite')
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    dummy_class_client = DummyClassClient()
    ordered_test_method_list = ['test_dummy_method_2', 'test_dummy_method_1']
    test_suite = ManualTestSuite(dummy_class_client, ordered_test_method_list, unattended=args.unattended)
