#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import os
import rospy
import rospkg
import Tkinter as tk
import tkSimpleDialog as simpledialog
import tkMessageBox as messagebox

class SrUrLoadCalibration():
    def __init__(self):
        os.environ['ARM_IPS'] = "192.168.1.1\n192.168.2.1" # Should be set by aurora, temporary for development
        self.ur_arm_ssh_username="root"
        self.ur_arm_ssh_password="easybot"
        rospack = rospkg.RosPack()
        self.sr_ur_arm_calibration_root =  rospack.get_path('sr_ur_arm_calibration_loader')
        self.arm_pointer_folder = os.path.join(self.sr_ur_arm_calibration_root, 'config')
        self.arm_calibrations_folder = os.path.join(self.sr_ur_arm_calibration_root, 'calibrations')
        self.default_kinematics_config = os.path.join(rospack.get_path('ur_description'), 'config', 'ur10_default.yaml')
        self.arm_ips = os.getenv('ARM_IPS').split('\n')
        self.first_gui_instance = True
        self.setup_folders()
	# self.check_arm_serial_in_file('192.168.1.1')

        while not rospy.is_shutdown():
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
        client.close()
        for line in stdout:
            arm_serial_number = line.strip('\n')
        return arm_serial_number

    def read_file(self, file_path):
        f = open(file_path, "r")
	try:
            file_content = f.read()
        except:
            rospy.logerr("Unexpected error %s reading %s in node %s", sys.exc_info()[0], file_path, rospy.get_name())
        f.close()
        return file_content

    def check_calibration_pointer_file(self, arm_ip, arm_serial):
        arm_serial_file = os.path.join(self.arm_pointer_folder, str(arm_ip) + '_serial')
        print self.read_file(arm_serial_file)

    def check_arm_calibration_exists(self, arm_serial):
        arm_calibration_file = os.path.join(self.arm_calibrations_folder, arm_serial + '.yaml')
        print arm_calibration_file
        if os.path.isfile(arm_calibration_file):
            return True
        else:
            return False

    def generate_new_calibration(arm_ip, arm_serial):
        if self.first_gui_instance:
            ROOT = tk.Tk()
            ROOT.withdraw()
            self.first_gui_instance = False
        question_string = "No calibration detected for arm at " + arm_ip + ". Do you want to generate one?"
        answer = messagebox.askokcancel("Question", question_string)
        if True == answer:
           self.start_calibration(arm_ip, arm_serial)
           self.set_calibration_file_to_serial(arm_ip, arm_serial)
        else: 
           self.set_calibration_file_to_default(arm_ip)

    def write_pointer_file(pointer_file, calibration_file_location):
        f = open(pointer_file, "w+")
        f.write(calibration_file_location)
        f.close

    def set_calibration_file_to_serial(arm_ip, arm_serial):
        pointer_file_location = os.path.join(self.arm_pointer_folder, arm_ip)
        calibration_file_location = os.path.join(self.arm_calibrations_folder, arm_serial + ".yaml")
        self.write_pointer_file(pointer_file_location, calibration_file_location)

    def set_calibration_file_to_default(arm_ip):
        pointer_file_location = os.path.join(self.arm_pointer_folder, arm_ip)
        self.write_pointer_file(pointer_file_location, self.default_kinematics_config)

    def start_calibration(arm_ip, arm_serial):
        output_file = os.path.join(self.arm_calibrations_folder, arm_serial + ".yaml")
        command = "rosrun ur_calibration calibration_correction robot_ip:=" + arm_ip + " output_filename:=" + output_file
        self.calibration_node = subprocess.Popen(command , stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

    def run(self):
        for arm_ip in self.arm_ips:
            #arm_serial = self.get_serial_from_arm(arm_ip)
            arm_serial = '1234567'
            if not self.check_arm_calibration_exists(arm_serial):
                self.generate_new_arm_calibration(arm_ip, arm_serial)
            self.update_arm_calibration_pointer_file(arm_ip, arm_serial)


if __name__ == "__main__":
    rospy.init_node("sr_ur_load_calibration")
    sr_ur_load_calibration = SrUrLoadCalibration()

