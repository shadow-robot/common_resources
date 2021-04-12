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
import datetime
from sensor_msgs.msg import JointState
from sr_utilities_common.aws_manager import AWS_Manager


class WearLogger():

    def __init__(self):
        self.aws_manager = AWS_Manager()
        self._first_run = True
        self._previous_values = {}
        self._current_values = {}
        self._threshold = 0.0175
        self._hand_serial = rospy.get_param("~hand_serial")
        self._log_file_path = rospkg.RosPack().get_path('sr_wear_logger') + "/" + self._hand_serial + "/"
        self._log_file_name = "wear_data.yaml"
        self._init_log()

        if self._hand_serial != "":
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
        if self._first_run:
            self._previous_values = hand_data
            self._first_run = False
        for key, value in hand_data.items():
            value_difference = abs(self._previous_values[key] - value)
            if value_difference > self._threshold:
                self._current_values[key] += round(value_difference, 5)
        self._previous_values = hand_data

    def _get_latest_file_name_from_AWS(self):

        files = self.aws_manager.get_bucket_structure_with_prefix("shadowrobot.benchmarks", self._hand_serial)
        if files == None:
            return ""

        files.sort(key=lambda x: x['LastModified'])
        last_file_path = files[-1]['Key']
        latest_file_name_from_AWS = last_file_path[last_file_path.find("/")+1:]

        return latest_file_name_from_AWS

    def _upload_to_AWS(self, event):
        
        success = False

        orignal_file = self._log_file_path + self._log_file_name
        now = datetime.datetime.now()
        dated_file = now.strftime("%Y-%m-%d-%H-%M-%S") + ".yaml"
        shutil.copy(orignal_file, self._log_file_path + dated_file)
        rospy.sleep(1)

        while os.stat(self._log_file_path + dated_file).st_size == 0:
            shutil.copy(orignal_file, self._log_file_path + dated_file)
            rospy.sleep(0.5)

        success = self.aws_manager.upload("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                          self._hand_serial, [dated_file])
        rospy.sleep(1)

        if os.path.exists(self._log_file_path + dated_file):
            os.remove(self._log_file_path + dated_file)

        return success

    def _download_from_AWS(self):
        success = False
        success = self.aws_manager.download("shadowrobot.benchmarks", rospkg.RosPack().get_path('sr_wear_logger'),
                                            self._hand_serial, [self._get_latest_file_name_from_AWS()])
        if success:
            rospy.sleep(1)
            latest_file_name_from_AWS = self._get_latest_file_name_from_AWS()
            shutil.copy(self._log_file_path + latest_file_name_from_AWS, self._log_file_path + self._log_file_name)
            rospy.sleep(1)
            if os.path.exists(self._log_file_path + latest_file_name_from_AWS):
                os.remove(self._log_file_path + latest_file_name_from_AWS)

        return success

    def _save_data_localy(self, event):
        if self._verify_data_empty():
            print("saving data locally")
            self._complete_data['total_angles_[rad]'] = self._current_values
            self._complete_data['total_time_[s]'] = rospy.get_rostime().secs
            f = open(self._log_file_path+self._log_file_name, 'w')
            yaml.safe_dump(self._complete_data, f)
            f.close()

    def _verify_data_empty(self):
        if len(self._current_values.keys()) == 0:
            return False
        return True

    def _load_data_from_yaml(self):
        self._current_values = self._complete_data['total_angles_[rad]']
        self._current_time = self._complete_data['total_time_[s]']
        with open(self._log_file_path+self._log_file_name, 'r') as f:
            self._complete_data = yaml.load(f, Loader=yaml.SafeLoader)
 
    def _update_with_higher_values(self, local_data, aws_data):
        for k, v in local_data.items():
            if isinstance(v, dict):
                local_data[k] = self._update_with_higher_values(aws_data.get(k, {}), v)
            else:
                local_data[k] = max(local_data[k], aws_data[k])
        return local_data

    def _update_log(self, local_file_path, aws_file_path):
        with open(local_file_path, 'r') as f_local:
            data_local = yaml.load(f_local, Loader=yaml.SafeLoader)
        with open(aws_file_path, 'r') as f_aws:
            data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)

        self._complete_data = self._update_with_higher_values(data_local, data_aws)
        self._save_data_localy(None)

    def _init_log(self):

        if not os.path.exists(self._log_file_path):
            os.makedirs(self._log_file_path)
            self._download_from_AWS()        

        if os.path.exists(self._log_file_path + self._log_file_name):
            rospy.loginfo("Comparing!")
            shutil.copy(self._log_file_path + self._log_file_name, self._log_file_path + "/wear_data_local.yaml")

            with open(self._log_file_path + "/wear_data_local.yaml", 'r') as f_local_copy:
                data_local_copy = yaml.load(f_local_copy, Loader=yaml.SafeLoader)

            self._download_from_AWS()

            with open(self._log_file_path + "/wear_data.yaml", 'r') as f_aws:
                data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)

            self._update_log(self._log_file_path + "/wear_data_local.yaml", self._log_file_path + "/wear_data.yaml")
            self._load_data_from_yaml()

            if os.path.exists(self._log_file_path + "/wear_data_local.yaml"):
                os.remove(self._log_file_path + "/wear_data_local.yaml")

        else:
            rospy.loginfo("Initiated a new log file!")
            msg = rospy.wait_for_message('/joint_states', JointState)

            self._current_values = dict.fromkeys(self._extract_hand_data(msg).keys(), 0.0)
            self._current_time = 0
            self._complete_data = dict()
            self._complete_data['total_angles_[rad]'] = self._current_values
            self._complete_data['total_time_[s]'] = 0

            with open(self._log_file_path+self._log_file_name, 'w') as f_init:
                yaml.safe_dump(self._complete_data, f_init)

            rospy.loginfo("Uploading!")
            self._upload_to_AWS(None)


if __name__ == "__main__":
    rospy.init_node('sr_wear_logger_node')
    logger = WearLogger()
    rospy.spin()
