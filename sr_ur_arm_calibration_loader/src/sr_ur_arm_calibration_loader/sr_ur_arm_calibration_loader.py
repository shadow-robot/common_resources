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

import os
import rospy
import roslaunch
import rospkg
import Tkinter as tk
import tkSimpleDialog as simpledialog
import tkMessageBox as messageBox
import paramiko
import sys
import yaml
from rosparam import upload_params


class SrUrLoadCalibration(object):
    def __init__(self, arm_info_in=[]):
        self._default_kinematics_config = ''
        self._arm_calibrations_folder = ''
        if [] == arm_info_in:
            rospy.logerr("No arms specified, cannot find arm calibration")
        self._arm_info_in = arm_info_in
        CONST_ARM_TYPE = self._arm_info_in[0]['arm_type'].lower()
        self._ur_arm_ssh_username = "root"
        self._ur_arm_ssh_password = "easybot"
        self._first_gui_instance = True
        self._rospack = rospkg.RosPack()
        if 'sr_ur_calibration' in self._rospack.list():
            CONST_SR_UR_ARM_CALIBRATION_ROOT = self._rospack.get_path('sr_ur_calibration')
        else:
            CONST_SR_UR_ARM_CALIBRATION_ROOT = self._rospack.get_path('sr_ur_arm_calibration_loader')
        self._arm_calibrations_folder = os.path.join(CONST_SR_UR_ARM_CALIBRATION_ROOT, 'calibrations')
        self._setup_folders()
        if 'e' in CONST_ARM_TYPE:
            self._default_kinematics_config = os.path.join(self._rospack.get_path('ur_e_description'),
                                                           'config', CONST_ARM_TYPE + '_default.yaml')
        else:
            self._default_kinematics_config = os.path.join(self._rospack.get_path('ur_description'),
                                                           'config', CONST_ARM_TYPE + '_default.yaml')
        if not os.path.isfile(self._default_kinematics_config):
            rospy.logerr('Arm type ' + CONST_ARM_TYPE + ' not recognised.')

    def _setup_folders(self):
        if not os.path.exists(self._arm_calibrations_folder):
            os.makedirs(self._arm_calibrations_folder)

    def _get_serial_from_arm(self, arm_ip):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(arm_ip, username=self._ur_arm_ssh_username, password=self._ur_arm_ssh_password)
        stdin, stdout, stderr = client.exec_command('cat /root/ur-serial')
        arm_serial_number = stdout.readline()
        client.close()
        if '' == arm_serial_number:
            rospy.logwarn("Could not retrieve arm serial number via SSH, arm will NOT be calibrated.")
            arm_serial_number = self._default_kinematics_config
        return arm_serial_number

    def _check_arm_calibration_exists(self, arm_serial):
        arm_calibration_file = os.path.join(self._arm_calibrations_folder, arm_serial + '.yaml')
        if os.path.isfile(arm_calibration_file):
            return True
        else:
            return False

    def _generate_new_arm_calibration(self, arm_ip, arm_serial):
        try:
            root = tk.Tk()
            root.withdraw()            
            question_string = "No calibration detected for arm at " + arm_ip + ". Do you want to generate one?"
        except:
            rospy.logerr("Cannot create graphical prompt. If this is running over SSH, are SSH graphics enabled?")
        answer = messageBox.askokcancel("Question", question_string)
        root.destroy()
        if answer is True:
            self._start_calibration(arm_ip, arm_serial)
            return True
        else:
            return False

    def _get_yaml(self, filename):
        with open(filename) as f:
            data = yaml.load(f)
        return data

    def _start_calibration(self, arm_ip, arm_serial):
        output_file = os.path.join(self._arm_calibrations_folder, arm_serial + ".yaml")
        pkg = 'ur_calibration'
        launch_file_name = 'calibration_correction.launch'
        roslaunch_file = os.path.join(self._rospack.get_path(pkg), 'launch', launch_file_name)
        robot_ip_arg = 'robot_ip:=' + arm_ip
        output_filename_arg = 'target_filename:=' + output_file
        cli_args = [roslaunch_file, output_filename_arg, robot_ip_arg]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        rospy.sleep(8)
        parent.shutdown()

    def get_calibration_files(self):
        arm_calibration_info_list = []
        for arm_info in self._arm_info_in:
            arm_ip = arm_info['ip_address']
            arm_side = arm_info['prefix']
            arm_serial = self._get_serial_from_arm(arm_ip)
            calibration_exists = self._check_arm_calibration_exists(arm_serial)
            if not calibration_exists:
                calibration_exists = self._generate_new_arm_calibration(arm_ip, arm_serial)
            if calibration_exists:
                calibration_file_location = os.path.join(self._arm_calibrations_folder, arm_serial + ".yaml")
            else:
                calibration_file_location = self._default_kinematics_config
            kinematics_config = self._get_yaml(calibration_file_location)
            upload_params('/' + arm_side + '_sr_ur_robot_hw', kinematics_config)
            arm_info_out = {}
            arm_info_out['prefix'] = arm_side
            arm_info_out['ip_address'] = arm_ip
            arm_info_out['arm_serial'] = arm_serial
            arm_info_out['kinematics_config'] = calibration_file_location
            arm_calibration_info_list.append(arm_info_out)
        return arm_calibration_info_list
