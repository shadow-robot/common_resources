#!/usr/bin/env python
import rospy
import os
import rospkg
import roslaunch

rospack = rospkg.RosPack()

arm_ip="192.168.1.1"
arm_serial="12345676"
output_file = os.path.join("/home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations", arm_serial + ".yaml")

pkg1 = 'ur_calibration'
launch_file = 'calibration_correction.launch'
arg1 = 'arm_ip:="' + arm_ip + '"'
arg2 = 'output_filename:="' + output_file + '"'

#cli_args1 = [pkg1, launch_file, arg1, arg2]
cli_args1 = [pkg1, launch_file, arg1, arg2]
#cli_args1 = ['pkg1', 'file1.launch', 'arg1:=arg1', 'arg2:=arg2']

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
roslaunch_args1 = cli_args1[2:]
launch_file = (roslaunch_file1, roslaunch_args1)
launch_file = ('/home/user/projects/shadow_robot/base/src/Universal_Robots_ROS_Driver/ur_calibration/launch/calibration_correction.launch', ['arm_ip:="192.168.1.1"', 'output_filename:="/home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations/12345676.yaml"'])

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

parent.start()



import os
import rospkg
import roslaunch
import rospy


rospack = rospkg.RosPack()

arm_ip="192.168.1.1"
arm_serial="12345676"
output_file = os.path.join("/home/user/projects/shadow_robot/base/src/common_resources/sr_ur_arm_calibration_loader/calibrations", arm_serial + ".yaml")

pkg = 'ur_calibration'
launch_file_name = 'calibration_correction.launch'
launch_file_components = [pkg, launch_file_name]
roslaunch_file = os.path.join(rospack.get_path(pkg), 'launch', launch_file_name)
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
# 3 seconds later
parent.shutdown()

import paramiko

def get_serial_from_arm(arm_ip):
import paramiko
arm_ip = "192.168.1.1"
ur_arm_ssh_username = "root"
ur_arm_ssh_password = "easybot"
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(arm_ip, username=ur_arm_ssh_username, password=ur_arm_ssh_password)
stdin, stdout, stderr = client.exec_command('cat /root/ur-serial')
print stdin, stdout, stderr
    client.close()
    for line in stdout:
        arm_serial_number = line.strip('\n')
    return arm_serial_number