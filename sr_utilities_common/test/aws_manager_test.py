#!/usr/bin/python

# Copyright 2021 Shadow Robot Company Ltd.
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
import rospkg
from std_msgs.msg import Bool
from sr_utilities_common.manual_test_suite import ManualTestSuite
from sr_utilities_common.aws_manager import AWS_Manager
import os

class Test_AWS_Manager(object):
    def __init__(self):
        self.aws_manager = AWS_Manager()

    def aws_get_bucket_structure_with_prefix(self):
        result_1 = self.aws_manager.get_bucket_structure_with_prefix("shadowrobot.benchmarks", None)
        result_2 = self.aws_manager.get_bucket_structure_with_prefix("shadowrobot.benchmarks", "test")
        return result_1 is None and result_2 is not None

    def aws_download(self):
        self.aws_manager.download("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_utilities_common'),
                                  'test', ["test.txt"])
        if os.path.exists("test.txt"):
            os.remove("test.txt")
            return True
        return False

    def aws_upload(self):
        self.aws_manager.upload("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_utilities_common'),
                                  'test', ["test.txt"])
        folder_list = self.aws_manager.get_bucket_structure_with_prefix("shadowrobot.benchmarks",'test')
        if folder_list is not None:
            for element in folder_list:
                if element['Key'] == 'test/test.txt':
                    return True
        return False

if __name__ == '__main__':
    rospy.init_node('aws_manager_test')

    test_aws_manager = Test_AWS_Manager()
    method_list = [#'aws_get_bucket_structure_with_prefix', 
                   'aws_download', 'aws_upload']
    test_suite = ManualTestSuite(test_aws_manager, method_list)

    
