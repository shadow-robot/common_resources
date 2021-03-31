#!/usr/bin/env python
#
# Copyright <Year> Shadow Robot Company Ltd.
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
import os
import yaml
import roslaunch
import requests
import shutil
from sensor_msgs.msg import JointState
from sr_utilities_common.awsmanager import AWS_Manager


class WearLogger():

    def _extractHandData(self, msg):
        hand_data = dict(zip(msg.name, msg.position))
        for key in hand_data.keys():
            if key.startswith("ra") or key.startswith("la"):
                hand_data.pop(key, None)
        return hand_data

    def callback(self, msg):
        hand_data = self._extractHandData(msg)
        updates = dict.fromkeys(hand_data.keys(), 0)

        if self.first_run:
            self.previousValues = hand_data
            self.first_run = False

        for key, value in hand_data.items():
            valueDifference = abs(self.previousValues[key] - value)
            if valueDifference > self.threshold:
                self.currentValues[key] += round(valueDifference, 5)
        self.previousValues = hand_data

    def _uploadToAWS(self, event):
        self.aws_manager.upload("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                "data", ["wear_data.yaml"])
        return rospy.get_param('aws_upload_succeeded')

    def _downloadFromAWS(self):
        self.aws_manager.download("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                  "data", ["wear_data.yaml"])
        return rospy.get_param('aws_download_succeeded')

    def _saveDataLocaly(self, event):
        if self._verifyData():
            f = open(self.logFilePath+self.logFileName, 'w')
            completeData = dict()
            completeData['total_angles'] = self.currentValues
            completeData['total_time'] = rospy.get_rostime().secs
            yaml.safe_dump(completeData, f)
            f.close()

    def __init__(self):
        self.aws_manager = AWS_Manager()
        self.first_run = True
        self.previousValues = {}
        self.currentValues = {}
        self.threshold = 0.0175
        self.logFilePath = "../data/"
        self.logFileName = "wear_data.yaml"
        self._initLog()
        rospy.Timer(rospy.Duration(5), self._saveDataLocaly)
        rospy.Timer(rospy.Duration(10), self._uploadToAWS)
        rospy.Subscriber('/joint_states', JointState, self.callback)

    def _verifyData(self):
        if len(self.currentValues.keys()) == 0:
            return False
        return True

    def _loadDataFromYAML(self):
        f = open(self.logFilePath+self.logFileName, 'r')
        completeData = yaml.load(f, Loader=yaml.SafeLoader)
        self.currentValues = completeData['total_angles']
        self.currentTime = completeData['total_time']
        f.close()

    def _update_with_bigger(self, dict1, dict2):
        for k, v in dict1.items():
            if isinstance(v, dict):
                dict1[k] = self._update_with_bigger(dict1.get(k, {}), v)
            else:
                dict1[k] = max(dict1[k], dict2[k])
        return dict1

    def _updateLog(self, local_file_path, aws_file_path):
        f_local = open(local_file_path, 'rw')
        data_local = yaml.load(f_local, Loader=yaml.SafeLoader)
        f_aws = open(aws_file_path, 'rw')
        rospy.sleep(1)
        data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)
        rospy.sleep(1)
        self.completeData = self._update_with_bigger(data_local, data_aws)
        self._saveDataLocaly(None)
        rospy.sleep(1)
        f_local.close()
        f_aws.close()

    def _initLog(self):
        if not os.path.exists(self.logFilePath):
            self._downloadFromAWS()

        if os.path.exists(self.logFilePath + self.logFileName):
            shutil.copyfile(self.logFilePath + self.logFileName, self.logFilePath + "/wear_data_local.yaml")
            self._downloadFromAWS()
            rospy.sleep(1)
            self._updateLog(self.logFilePath + "/wear_data_local.yaml", self.logFilePath + "/wear_data.yaml")
            rospy.sleep(1)
            self._loadDataFromYAML()

        else:
            f = open(self.logFilePath+self.logFileName, 'w')
            msg = rospy.wait_for_message('/joint_states', JointState)
            self.currentValues = dict.fromkeys(self._extractHandData(msg).keys(), 0.0)
            self.currentTime = 0
            completeData = dict()
            completeData['total_angles'] = self.currentValues
            completeData['total_time'] = 0
            yaml.safe_dump(completeData, f)
            rospy.sleep(1)
            f.close()
            self._uploadToAWS(None)

if __name__ == "__main__":
    rospy.init_node('sr_wear_logger_node')
    logger = WearLogger()
    rospy.spin()
