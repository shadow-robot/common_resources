#!/usr/bin/python

import rospy
import sys
import argparse


class ManualTestSuite(object):
    def __init__(self, tested_class, unattended=False):
        self.tested_class = tested_class
        self.unattended = unattended
        self.test_results = {'total': 0, 'passed': 0, 'failed': 0}
        self.color_codes = {'green': '\033[92m', 'red': '\033[91m',
                            'orange': '\033[93m', 'default': '\033[0m'}

    def create_test(self, command):
        if not self.unattended:
            raw_input(self.color_codes['orange'] +
                      'Test {}: {}. Press [RETURN] to continue...'.format(self.test_results['total'], command))
        self.test_results['total'] += 1
        exec('result = self.tested_class.{}'.format(command))
        if result:
            rospy.logwarn('{} test passed'.format(command))
            self.test_results['passed'] += 1
        else:
            rospy.logwarn('{} test failed!'.format(command))
            self.test_results['failed'] += 1

    def print_summary(self):
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

    def dummy_method(self):
        if 1 == self.dummy_variable:
            return True


class DummyClassClient(object):
    def __init__(self):
        self.dummy_class_instance = DummyClass()

    def test_dummy_method(self):
        return self.dummy_class_instance.dummy_method()

if __name__ == '__main__':
    rospy.init_node('manual_test_suite')
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    dummy_class_client = DummyClassClient()
    test_suite = ManualTestSuite(dummy_class_client, unattended=args.unattended)
    test_suite.create_test('test_dummy_method')
    test_suite.print_summary()
