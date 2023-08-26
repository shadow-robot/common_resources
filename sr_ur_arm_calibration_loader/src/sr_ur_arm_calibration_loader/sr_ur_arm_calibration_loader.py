#!/usr/bin/python3

# Copyright 2020-2022 Shadow Robot Company Ltd.
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
import socket
import paramiko
import yaml
from rosparam import upload_params
from paramiko.ssh_exception import BadHostKeyException, AuthenticationException, SSHException, NoValidConnectionsError
import rospy
import roslaunch
import rospkg


class SrUrLoadCalibrationExceptions(Exception):
    pass


class NoArmsSpecified(SrUrLoadCalibrationExceptions):
    pass


class ArmTypeNotRecognised(SrUrLoadCalibrationExceptions):
    pass


class DifferentArmTypes(SrUrLoadCalibrationExceptions):
    pass


def get_yaml(filename):
    with open(filename, encoding='UTF-8') as yaml_file:
        return yaml.safe_load(yaml_file)


class SrUrLoadCalibration:

    _CONST_UR_ARM_SSH_USERNAME = "root"
    _CONST_UR_ARM_SSH_PASSWORD = "easybot"

    def __init__(self, arm_info_in=None):
        self._default_kinematics_config = ''
        self._arm_calibrations_folder = ''

        if not arm_info_in:
            raise NoArmsSpecified("No arms specified, cannot find arm calibration")
        self._arm_info_in = arm_info_in
        arm_type = self._arm_info_in[0]['arm_type'].lower()
        if len(self._arm_info_in) > 1:
            for arm in self._arm_info_in:
                if arm['arm_type'].lower() != arm_type:
                    raise DifferentArmTypes("Different arm types specified. \
                                            Currently this node only supports arms of the same type")

        if 'sr_ur_calibration' in rospkg.RosPack().list():
            sr_ur_arm_calibration_root = rospkg.RosPack().get_path('sr_ur_calibration')
        else:
            sr_ur_arm_calibration_root = rospkg.RosPack().get_path('sr_ur_arm_calibration_loader')
        self._arm_calibrations_folder = os.path.join(sr_ur_arm_calibration_root, 'calibrations')
        self._setup_folders()
        self._default_kinematics_config = os.path.join(rospkg.RosPack().get_path('ur_description'),
                                                       'config', arm_type + '/default_kinematics.yaml')
        if not os.path.isfile(self._default_kinematics_config):
            raise ArmTypeNotRecognised('Cannot find default config for ' + arm_type)

    def _setup_folders(self):
        if not os.path.exists(self._arm_calibrations_folder):
            os.makedirs(self._arm_calibrations_folder)

    def _get_serial_from_arm(self, arm_ip):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        arm_serial_number = ''
        ssh_exception_message = ''
        try:
            client.connect(arm_ip, username=self._CONST_UR_ARM_SSH_USERNAME,
                           password=self._CONST_UR_ARM_SSH_PASSWORD, timeout=5.0)
            _, stdout, _ = client.exec_command('cat /root/ur-serial')
            arm_serial_number = stdout.readline()
            client.close()
        except NoValidConnectionsError as exception:
            ssh_exception_message = f"Failed to SSH into arm - {exception}"
        except (BadHostKeyException, AuthenticationException, SSHException, socket.error) as exception:
            ssh_exception_message = f"Failed to SSH into arm - {exception}"

        if arm_serial_number == '':
            rospy.logwarn(f"Could not retrieve arm serial number.{ssh_exception_message}"
                          " Arm will NOT be calibrated. Ignore if running URSim.")
        return arm_serial_number

    def _arm_calibration_exists(self, arm_serial):
        return os.path.isfile(os.path.join(self._arm_calibrations_folder, arm_serial + '.yaml'))

    def _start_calibration(self, arm_ip, arm_serial):
        output_file = os.path.join(self._arm_calibrations_folder, arm_serial + ".yaml")
        pkg = 'ur_calibration'
        launch_file_name = 'calibration_correction.launch'
        roslaunch_file = os.path.join(rospkg.RosPack().get_path(pkg), 'launch', launch_file_name)
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
            if arm_serial != '':
                if not self._arm_calibration_exists(arm_serial):
                    self._start_calibration(arm_ip, arm_serial)
                calibration_file_location = os.path.join(self._arm_calibrations_folder, arm_serial + ".yaml")
            else:
                calibration_file_location = self._default_kinematics_config
            kinematics_config = get_yaml(calibration_file_location)
            upload_params('/' + arm_side + '_sr_ur_robot_hw', kinematics_config)
            arm_info_out = {}
            arm_info_out['prefix'] = arm_side
            arm_info_out['ip_address'] = arm_ip
            arm_info_out['arm_serial'] = arm_serial
            arm_info_out['kinematics_config'] = calibration_file_location
            arm_calibration_info_list.append(arm_info_out)
        return arm_calibration_info_list
