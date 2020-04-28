#!/usr/bin/env python
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


class SrUrLoadCalibration():
    def __init__(self):
        os.environ['ARM_IPS'] = "192.168.1.1\n192.168.2.1"  # Should be set by aurora, temporary for development
        self.ur_arm_ssh_username="root"
        self.ur_arm_ssh_password="easybot"
        self.rospack = rospkg.RosPack()
        self.sr_ur_arm_calibration_root =  self.rospack.get_path('sr_ur_arm_calibration_loader')
        self.arm_pointer_folder = os.path.join(self.sr_ur_arm_calibration_root, 'config')
        self.arm_calibrations_folder = os.path.join(self.sr_ur_arm_calibration_root, 'calibrations')
        self.default_kinematics_config = os.path.join(self.rospack.get_path('ur_description'), 'config', 'ur10_default.yaml')
        self.arm_ips = os.getenv('ARM_IPS').split('\n')
        self.first_gui_instance = True
        self.setup_folders()
        # self.check_arm_serial_in_file('192.168.1.1')
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down %s", rospy.get_name())

    def setup_folders(self):
        for folder in self.arm_pointer_folder, self.arm_calibrations_folder:
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

    def read_file(self, file_path):
        f = open(file_path, "r")
        try:
            file_content = f.read()
        except:
            rospy.logerr("Unexpected error %s reading %s in node %s", sys.exc_info()[0], file_path, rospy.get_name())
        f.close()
        return file_content

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

    def write_pointer_file(self, pointer_file, calibration_file_location):
        f = open(pointer_file, "w+")
        f.write(calibration_file_location)
        f.close

    def set_pointer_file_to_serial(self, arm_ip, arm_serial):
        pointer_file_location = os.path.join(self.arm_pointer_folder, arm_ip)
        calibration_file_location = os.path.join(self.arm_calibrations_folder, arm_serial + ".yaml")
        self.write_pointer_file(pointer_file_location, calibration_file_location)

    def set_pointer_file_to_default(self, arm_ip):
        pointer_file_location = os.path.join(self.arm_pointer_folder, arm_ip)
        self.write_pointer_file(pointer_file_location, self.default_kinematics_config)

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

    def run(self):
        for arm_ip in self.arm_ips:
            calibration_generated = False
            arm_serial = self.get_serial_from_arm(arm_ip)
            #arm_serial = '1234567'
            if not self.check_arm_calibration_exists(arm_serial):
                calibration_generated = self.generate_new_arm_calibration(arm_ip, arm_serial)
            if calibration_generated:
                self.set_pointer_file_to_serial(arm_ip, arm_serial)
            else:
                self.set_pointer_file_to_default(arm_ip)


if __name__ == "__main__":
    rospy.init_node("sr_ur_load_calibration")
    sr_ur_load_calibration = SrUrLoadCalibration()
