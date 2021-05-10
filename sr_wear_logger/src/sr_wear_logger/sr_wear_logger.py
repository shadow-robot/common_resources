#!/usr/bin/env python3
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

from __future__ import absolute_import
from builtin import round
import rospy
import rospkg
import os
import re
import yaml
import shutil
import datetime
from sensor_msgs.msg import JointState
from sr_utilities_common.aws_manager import AWS_Manager
from rospy import ROSException

THRESHOLD = 0.0175
BENCHMARK_NAME = "shadowrobot.wear"


class SrWearLogger():
    def __init__(self, hand_serial, hand_id, aws_save_period, local_save_period):
        self._running = False
        self._first_run = True
        self._previous_values = {}
        self._current_values = {}
        self._complete_data = 0
        self._time_counter = 0

        self._hand_serial = hand_serial
        self._hand_id = hand_id
        self._local_save_period = local_save_period
        self._aws_save_period = aws_save_period

        self._log_file_path = rospkg.RosPack().get_path('sr_wear_logger') + "/" + str(self._hand_serial) + "/"
        self._log_file_name = "wear_data.yaml"
        self._clear_from_files()

    def _clear_from_files(self):
        pattern = r"\d\d\d\d-\d\d-\d\d-\d\d-\d\d-\d\d.yaml"
        if (os.path.exists(self._log_file_path)):
            for file in os.listdir(self._log_file_path):
                if bool(re.match(pattern, file)):
                    os.remove(cls.path_to_test_folder + "/" + file)

    def check_parameters(self):
        check_serial = self._hand_serial is not None and self._hand_serial != ""
        check_hand_id = self._hand_id == "rh" or self._hand_id == "lh"
        check_aws = self._aws_save_period is not None and type(self._aws_save_period) == int
        check_local = self._local_save_period is not None and type(self._local_save_period) == int
        return check_serial and check_hand_id and check_aws and check_local

    def run(self):
        if self.check_parameters():
            self.aws_manager = AWS_Manager()
            self._verify_log_structure()
            self.t_local = rospy.Timer(rospy.Duration(self._local_save_period), self._save_data_localy)
            self.t_aws = rospy.Timer(rospy.Duration(self._aws_save_period), self._upload_to_AWS)
            rospy.Subscriber('/joint_states', JointState, self._callback)
            rospy.loginfo(rospy.get_name() + " SrWearLogger initialized!")
            self._running = True
        else:
            rospy.logwarn(rospy.get_name() + " Can't run SrWearLogger. Wrong parameters!")
            rospy.signal_shutdown("")

    def _verify_log_structure(self):
        if not os.path.exists(self._log_file_path):
            os.makedirs(self._log_file_path)
        if os.path.exists(self._log_file_path + self._log_file_name):
            self._sync_and_update_log_file()
        else:
            self._generate_log_file()

    def _sync_and_update_log_file(self):
        error_msg = " Log file path does not exist!"
        if os.path.exists(self._log_file_path + self._log_file_name):
            shutil.copy(self._log_file_path + self._log_file_name, self._log_file_path + "/wear_data_local.yaml")
            if self._check_download_from_AWS_successfull():
                local_file_path = self._log_file_path + "/wear_data_local.yaml"
                aws_file_path = self._log_file_path + "/wear_data.yaml"
                try:
                    with open(local_file_path, 'r') as f_local, open(aws_file_path, 'r') as f_aws:
                        data_local = yaml.load(f_local, Loader=yaml.SafeLoader)
                        data_aws = yaml.load(f_aws, Loader=yaml.SafeLoader)
                        self._complete_data = self._update_with_higher_values(data_local, data_aws)
                    if self._complete_data is not None:
                        self._current_values = self._complete_data['total_angles_[rad]']
                        self._current_time = self._complete_data['total_time_[s]']
                        self._save_data_localy()
                        self._upload_to_AWS()
                    else:
                        error_msg = " Data is empty!"
                except Exception as e:
                    error_msg = " No local/AWS log file found!"

            if os.path.exists(self._log_file_path + "/wear_data_local.yaml"):
                os.remove(self._log_file_path + "/wear_data_local.yaml")
        else:
            rospy.logwarn(rospy.get_name() + " Failed to sync and update log file." + error_msg)

    def _generate_log_file(self):
        rospy.loginfo(rospy.get_name() + " Waiting for /joint_states topic to create a log file..")
        msg = rospy.wait_for_message('/joint_states', JointState)

        self._current_values = dict.fromkeys(list(self._extract_hand_data(msg).keys()), 0.0)
        self._current_time = rospy.get_rostime().secs
        self._complete_data = dict()
        self._complete_data['total_angles_[rad]'] = self._current_values
        self._complete_data['total_time_[s]'] = self._current_time

        if self._save_data_localy() and self._upload_to_AWS():
            rospy.loginfo(rospy.get_name() + " New log file created!")
        else:
            rospy.logwarn(rospy.get_name() + " Could not create a new logfile. Failed to save data!")

    def _check_download_from_AWS_successfull(self):
        success = False
        success = self.aws_manager.download(BENCHMARK_NAME, rospkg.RosPack().get_path('sr_wear_logger'),
                                            self._hand_serial, [self._get_latest_file_name_from_AWS()])
        if success:
            latest_file_name_from_AWS = self._get_latest_file_name_from_AWS()
            if os.path.exists(self._log_file_path + latest_file_name_from_AWS):
                shutil.copy(self._log_file_path + latest_file_name_from_AWS, self._log_file_path + self._log_file_name)
                rospy.sleep(0.5)
                os.remove(self._log_file_path + latest_file_name_from_AWS)
        return success

    def _get_latest_file_name_from_AWS(self):
        files = self.aws_manager.get_bucket_structure_with_prefix(BENCHMARK_NAME, self._hand_serial)
        if files is None:
            return ""
        files.sort(key=lambda x: x['LastModified'])
        last_file_path = files[-1]['Key']
        latest_file_name_from_AWS = last_file_path[last_file_path.find("/")+1:]
        return latest_file_name_from_AWS

    def _update_with_higher_values(self, local_data, aws_data):
        try:
            for k, v in list(local_data.items()):
                if isinstance(v, dict):
                    local_data[k] = self._update_with_higher_values(aws_data.get(k, {}), v)
                else:
                    local_data[k] = max(local_data[k], aws_data[k])
            return local_data
        except Exception as e:
            rospy.logwarn(rospy.get_name() + " Could not perform update. " + str(e))
            return None

    def _save_data_localy(self, event=None):
        success = False
        if not self._data_is_empty():
            self._complete_data['total_angles_[rad]'] = self._current_values
            self._complete_data['total_time_[s]'] += rospy.get_rostime().secs - self._time_counter
            self._time_counter = rospy.get_rostime().secs
            try:
                with open(self._log_file_path+self._log_file_name, 'w') as f:
                    yaml.safe_dump(self._complete_data, f)
                success = True
            except Exception as e:
                rospy.logwarn(rospy.get_name() + " Failed to sava data. " + str(e))
        return success

    def _data_is_empty(self):
        is_empty = False
        if len(list(self._current_values.keys())) == 0:
            is_empty = True
        if self._complete_data is None:
            is_empty = True
        return is_empty

    def _upload_to_AWS(self, event=None):
        success = False
        orignal_file = self._log_file_path + self._log_file_name
        now = datetime.datetime.now()
        dated_file = now.strftime("%Y-%m-%d-%H-%M-%S") + ".yaml"
        shutil.copy(orignal_file, self._log_file_path + dated_file)
        rospy.sleep(0.5)
        while os.stat(self._log_file_path + dated_file).st_size == 0:
            shutil.copy(orignal_file, self._log_file_path + dated_file)
            rospy.sleep(0.5)

        success = self.aws_manager.upload(BENCHMARK_NAME, rospkg.RosPack().get_path('sr_wear_logger'),
                                          self._hand_serial, [dated_file])
        rospy.sleep(0.5)
        if os.path.exists(self._log_file_path + dated_file):
            os.remove(self._log_file_path + dated_file)
        return success

    def _callback(self, msg):
        hand_data = self._extract_hand_data(msg)
        updates = dict.fromkeys(list(hand_data.keys()), 0.0)
        if self._first_run:
            self._previous_values = hand_data
            self._first_run = False
        for key, value in list(hand_data.items()):
            value_difference = abs(self._previous_values[key] - value)
            if value_difference > THRESHOLD:
                self._current_values[key] += round(value_difference, 5)
        self._previous_values = hand_data

    def _extract_hand_data(self, msg):
        hand_data = dict(list(zip(msg.name, msg.position)))
        for key in list(hand_data.keys()):
            if not key.startswith(self._hand_id):
                hand_data.pop(key, None)
        return hand_data

    def stop(self):
        if self._running:
            self.t_local.shutdown()
            self.t_aws.shutdown()
        else:
            rospy.logwarn(rospy.get_name() + " SrWearLogger is not running!")


if __name__ == "__main__":
    rospy.init_node('sr_wear_logger_node')

    hand_serial = rospy.get_param("~hand_serial", "")
    hand_id = rospy.get_param("~hand_id", "rh")
    local_save_period = rospy.get_param("~local_save_period", 2)
    aws_save_period = rospy.get_param("~aws_save_period", 10)

    logger = SrWearLogger(hand_serial, hand_id, aws_save_period, local_save_period)
    logger.run()

    rospy.spin()
