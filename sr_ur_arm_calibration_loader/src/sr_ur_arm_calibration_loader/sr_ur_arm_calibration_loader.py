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


class SrUrLoadCalibration(object):
    def __init__(self, arm_ip_list = []):
        if [] == arm_ip_list:
            rospy.logerr("No arm IPs specified, cannot find arm calibration")
        self.arm_is = arm_ip_list
        self.ur_arm_ssh_username="root"
        self.ur_arm_ssh_password="easybot"
        self.rospack = rospkg.RosPack()
        self.sr_ur_arm_calibration_root =  self.rospack.get_path('sr_ur_arm_calibration_loader')
        self.arm_pointer_folder = os.path.join(self.sr_ur_arm_calibration_root, 'config')
        self.arm_calibrations_folder = os.path.join(self.sr_ur_arm_calibration_root, 'calibrations')
        self.default_kinematics_config = os.path.join(self.rospack.get_path('ur_description'), 'config', 'ur10_default.yaml')
        self.first_gui_instance = True
        self.setup_folders()

    def setup_folders(self):
        for folder in self.arm_calibrations_folder:
            if not os.path.exists(folder):
                os.makedirs(folder)

    def get_serial_from_arm(self, arm_ip):
        client = paramiko.SSHClient()   
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(arm_ip, username=self.ur_arm_ssh_username, password=self.ur_arm_ssh_password)
        stdin, stdout, stderr = client.exec_command('cat /root/ur-serial')
        arm_serial_number = stdout.readline()
        client.close()
        if arm_serial_number == '':
            rospy.logwarn("Could not retrieve arm serial number via SSH")
        return arm_serial_number

    def check_arm_calibration_exists(self, arm_serial):
        arm_calibration_file = os.path.join(self.arm_calibrations_folder, arm_serial + '.yaml')
        print arm_calibration_file
        if os.path.isfile(arm_calibration_file):
            return True
        else:
            return False

    def generate_new_arm_calibration(self, arm_ip, arm_serial):
        try:
            if self.first_gui_instance:
                root = tk.Tk()
                root.withdraw()
                self.first_gui_instance = False
            question_string = "No calibration detected for arm at " + arm_ip + ". Do you want to generate one?"
        except:
            print "Cannot create graphical prompt. If this is running over SSH, are SSH graphics enabled?"
        answer = messageBox.askokcancel("Question", question_string)
        if True == answer:
            self.start_calibration(arm_ip, arm_serial)
            self.set_pointer_file_to_serial(arm_ip, arm_serial)
            return True
        else: 
            self.set_calibration_file_to_default(arm_ip)
            return False


    def start_calibration(self, arm_ip, arm_serial):
        output_file = os.path.join(self.arm_calibrations_folder, arm_serial + ".yaml")
        pkg = 'ur_calibration'
        launch_file_name = 'calibration_correction.launch'
        roslaunch_file = os.path.join(self.rospack.get_path(pkg), 'launch', launch_file_name)
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
        for arm_ip in self.arm_ips:
            arm_serial = self.get_serial_from_arm(arm_ip)
            calibration_exists = self.check_arm_calibration_exists(arm_serial)
            if not calibration_exists:
                calibration_exists = self.generate_new_arm_calibration(arm_ip, arm_serial)
            if calibration_exists:
                calibration_file_location = os.path.join(self.arm_calibrations_folder, arm_serial + ".yaml")
            else:
                calibration_file_location = self.default_kinematics_config
            arm_calibration_info_list.append((arm_ip, arm_serial, calibration_file_location))
        return arm_calibration_info_list


