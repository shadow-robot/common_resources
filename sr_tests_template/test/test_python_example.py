#!/usr/bin/env python3

# Copyright 2015, 2022 Shadow Robot Company Ltd.
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

from unittest import TestCase
import rospy
import rostest


PKG = "sr_tests_template"


class TestPythonExample(TestCase):
    def setUp(self):
        self.example_shared_resource = None

    def test_headless_gazebo(self):
        expected_value = 0
        tested_value = 0

        self.assertEqual(expected_value, tested_value)


if __name__ == "__main__":
    rospy.init_node('test_python_example', anonymous=True)
    rostest.rosrun(PKG, 'test_python_example', TestPythonExample)
