#!/usr/bin/env python
#
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
import os
import yaml
import roslaunch
import requests
import shutil
from sensor_msgs.msg import JointState
from sr_utilities_common.aws_manager import AWS_Manager


class WearLogger():

    def __init__(self):
        self.aws_manager = AWS_Manager()
        self.first_run = True
        self.previous_values = {}
        self.current_values = {}
        self.threshold = 0.0175
        self.log_file_path = rospack.get_path('sr_wear_logger') + "/data"
        self.log_file_name = "wear_data.yaml"
        self._init_log()
        rospy.Timer(rospy.Duration(2), self._save_data_localy)
        rospy.Timer(rospy.Duration(10), self._upload_to_AWS)
        rospy.Subscriber('/joint_states', JointState, self.callback)

    def _extract_hand_data(self, msg):
        hand_data = dict(zip(msg.name, msg.position))
        for key in hand_data.keys():
            if key.startswith("ra") or key.startswith("la"):
                hand_data.pop(key, None)
        return hand_data

    def callback(self, msg):
        hand_data = self._extract_hand_data(msg)
        updates = dict.fromkeys(hand_data.keys(), 0)
        if self.first_run:
            self.previous_values = hand_data
            self.first_run = False
        for key, value in hand_data.items():
            value_difference = abs(self.previous_values[key] - value)
            if value_difference > self.threshold:
                self.current_values[key] += round(value_difference, 5)
        self.previous_values = hand_data

    def _upload_to_AWS(self, event):
        return self.aws_manager.upload("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                       "data", ["wear_data.yaml"])

    def _download_from_AWS(self):
        return self.aws_manager.download("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                         "data", ["wear_data.yaml"])

    def _save_data_localy(self, event):
        if self._verify_data_empty():
            print("Saving file localy")
            self.complete_data['total_angles_[rad]'] = self.current_values
            self.complete_data['total_time_[s]'] = rospy.get_rostime().secs
            try:
                f = open(self.log_file_path+self.log_file_name, 'w')
                yaml.safe_dump(self.complete_data, f)
                f.close()
            except FileNotFoundError as e:
                pass

    def _verify_data_empty(self):
        if len(self.current_values.keys()) == 0:
            return False
        return True

    def _loadDataFromYAML(self):
        self.current_values = complete_data['total_angles_[rad]']
        self.current_time = complete_data['total_time_[s]']
        try:
            f = open(self.log_file_path+self.log_file_name, 'r')        
            complete_data = yaml.load(f, Loader=yaml.SafeLoader)
            f.close()
        except FileNotFoundError as e:
            pass


    def _update_with_higher_values(self, local_data, aws_data):
        for k, v in local_data.items():
            if isinstance(v, dict):
                local_data[k] = self._update_with_higher_values(aws_data.get(k, {}), v)
            else:
                local_data[k] = max(local_data[k], aws_data[k])
        return local_data

    def _update_log(self, local_file_path, aws_file_path):
        try:
            f_local = open(local_file_path, 'rw')
            data_local = yaml.load(f_local, Loader=yaml.SafeLoader)
            f_aws = open(aws_file_path, 'rw')
            rospy.sleep(1)
            data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)
            rospy.sleep(1)
            self.complete_data = self._update_with_higher_values(data_local, data_aws)
            self._save_data_localy(None)
            rospy.sleep(1)
            f_local.close()
            f_aws.close()
        except FileNotFoundError as e:
            pass

    def _init_log(self):
        if not os.path.exists(self.log_file_path):
            self._download_from_AWS()

        if os.path.exists(self.log_file_path + self.log_file_name):
            try:
                shutil.copyfile(self.log_file_path + self.log_file_name, self.log_file_path + "/wear_data_local.yaml")
                f_local_copy = open(self.log_file_path + "/wear_data_local.yaml", 'rw')
                data_local_copy = yaml.load(f_local_copy, Loader=yaml.SafeLoader)
                print("Local copy")
                print(data_local_copy)
                rospy.sleep(1)
                self._download_from_AWS()
                f_aws = open(self.log_file_path + "/wear_data.yaml", 'rw')
                data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)
                print("AWS copy")
                print(data_aws)
                rospy.sleep(1)            
                self._update_log(self.log_file_path + "/wear_data_local.yaml", self.log_file_path + "/wear_data.yaml")
                print("Current_data")
                print(self.complete_data)            
                rospy.sleep(1)
            except FileNotFoundError as e:
                pass
            self._loadDataFromYAML()

        else:
            self.current_values = dict.fromkeys(self._extract_hand_data(msg).keys(), 0.0)
            self.current_time = 0
            self.complete_data = dict()
            self.complete_data['total_angles_[rad]'] = self.current_values
            self.complete_data['total_time_[s]'] = 0
            
            try:
                f = open(self.log_file_path+self.log_file_name, 'w')
                msg = rospy.wait_for_message('/joint_states', JointState)
                yaml.safe_dump(self.complete_data, f)
                rospy.sleep(1)
                f.close()
                self._upload_to_AWS(None)
            except FileNotFoundError as e:
                pass


if __name__ == "__main__":
    rospy.init_node('sr_wear_logger_node')
    logger = WearLogger()
    rospy.spin()
