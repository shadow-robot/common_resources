#!/usr/bin/python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

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
    def __init__(self, arm_info_in = []):
        if [] == arm_info_in:
            rospy.logerr("No arms specified, cannot find arm calibration")
        self._arm_info_in = arm_info_in
        self.ur_arm_ssh_username="root"
        self.ur_arm_ssh_password="easybot"
        self._rospack = rospkg.RosPack()
        if 'sr_ur_calibration' in self._rospack.list():
            sr_ur_arm_calibration_root = self._rospack.get_path('sr_ur_calibration')
        else:
            sr_ur_arm_calibration_root = self._rospack.get_path('sr_ur_arm_calibration_loader')
        self._arm_calibrations_folder = os.path.join(sr_ur_arm_calibration_root, 'calibrations')
        self._setup_folders()
        self._default_kinematics_config = os.path.join(self._rospack.get_path('ur_description'), 'config', 'ur10_default.yaml')
        self._first_gui_instance = True

    def _setup_folders(self):
        if not os.path.exists(self._arm_calibrations_folder):
            os.makedirs(self._arm_calibrations_folder)

    def _get_serial_from_arm(self, arm_ip):
        client = paramiko.SSHClient()   
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(arm_ip, username=self.ur_arm_ssh_username, password=self.ur_arm_ssh_password)
        stdin, stdout, stderr = client.exec_command('cat /root/ur-serial')
        arm_serial_number = stdout.readline()
        client.close()
        if arm_serial_number == '':
            rospy.logwarn("Could not retrieve arm serial number via SSH")
        return arm_serial_number

    def _check_arm_calibration_exists(self, arm_serial):
        arm_calibration_file = os.path.join(self._arm_calibrations_folder, arm_serial + '.yaml')
        if os.path.isfile(arm_calibration_file):
            return True
        else:
            return False

    def _generate_new_arm_calibration(self, arm_ip, arm_serial):
        try:
            if self._first_gui_instance:
                root = tk.Tk()
                root.withdraw()
                self._first_gui_instance = False
            question_string = "No calibration detected for arm at " + arm_ip + ". Do you want to generate one?"
        except:
            rospy.logerr("Cannot create graphical prompt. If this is running over SSH, are SSH graphics enabled?")
        answer = messageBox.askokcancel("Question", question_string)
        if True == answer:
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
            arm_ip = arm_info[1]
            arm_side = arm_info[0]
            rospy.loginfo('arm_ip: ' + arm_ip)
            rospy.loginfo('arm_side: ' + arm_side)
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
            arm_info = {}
            arm_info['arm_side'] = arm_side
            arm_info['arm_ip'] = arm_ip
            arm_info['arm_serial'] = arm_serial
            arm_info['kinematics_config'] = calibration_file_location
            arm_calibration_info_list.append(arm_info)
        return arm_calibration_info_list


