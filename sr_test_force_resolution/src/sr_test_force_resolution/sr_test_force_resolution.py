#!/usr/bin/python
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
import copy
import termios
import sys
import csv
import datetime
from pynput import keyboard
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_controllers_tools.sr_controller_helper import ControllerHelper
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, SwitchControllerRequest)


class ControllerStateMonitor():
    def __init__(self, name):
        self.name = name
        self.enable_output = False
        self.subscriber = None
        self.output_controller_state_keys = ['timestamp', 'command', 'setpoint', 'process_value', 'process_value_dot']
        self.output_controller_state = {}
        self.initialise_output_dictionary()

    def initialise_output_dictionary(self):
        for key in self.output_controller_state_keys:
            self.output_controller_state[key] = []

    def unsubscribe_all(self):
        self.subscriber.unsubscribe()


class TestForceResolution():
    def __init__(self, keyboard_control=True, side="right"):
        self.active_tests = []
        self._controller_subscribers = {}
        self._last_joint_state = JointState()
        self._output_jointstate = {}
        self._hand_prefix = side[0] + 'h_'
        self._output_jointstate_keys = ['timestamp', 'position', 'velocity', 'effort']
        self._fingers_with_j0 = ['ff', 'mf', 'rf', 'lf']
        self._mode = ''
        self._j0_position_controllers = ["sh_{0}{1}j0_position_controller".format(self._hand_prefix, joint) for joint in self._fingers_with_j0]
        self.requested_joints = []
        # burning test via PWM - when uncalibrated we still want to run burn test via PWM
        #                      - for each joint in each finger, apply pwm of X
        self._joint_states_zero = {'rh_FFJ1': 0, 'rh_FFJ2': 0, 'rh_FFJ3': 0, 'rh_FFJ4': 0,
                                   'rh_MFJ1': 0, 'rh_MFJ2': 0, 'rh_MFJ3': 0, 'rh_MFJ4': 0,
                                   'rh_RFJ1': 0, 'rh_RFJ2': 0, 'rh_RFJ3': 0, 'rh_RFJ4': 0,
                                   'rh_LFJ1': 0, 'rh_LFJ2': 0, 'rh_LFJ3': 0, 'rh_LFJ4': 0, 'rh_LFJ5': 0,
                                   'rh_THJ1': 0, 'rh_THJ2': 0, 'rh_THJ3': 0, 'rh_THJ4': 0, 'rh_THJ5': 0,
                                   'rh_WRJ1': 0, 'rh_WRJ2': 0}
        self._joint_ranges = {'TH':
                              {
                                  '5': {'min': -60, 'max': 60},
                                  '4': {'min': 0,   'max': 70},
                                  '3': {'min': -12, 'max': 12},
                                  '2': {'min': -40, 'max': 40},
                                  '1': {'min': -15, 'max': 90},
                                  },
                              'FF':
                              {
                                  '4': {'min': -20, 'max': 20},
                                  '3': {'min': -15, 'max': 90},
                                  '2': {'min': 0,   'max': 90},
                                  '1': {'min': 0,   'max': 90},
                                  },
                              'MF':
                              {
                                  '4': {'min': -20, 'max': 20},
                                  '3': {'min': -15, 'max': 90},
                                  '2': {'min': 0,   'max': 90},
                                  '1': {'min': 0,   'max': 90},
                                  },
                              'RF':
                              {
                                  '4': {'min': -20, 'max': 20},
                                  '3': {'min': -15, 'max': 90},
                                  '2': {'min': 0,   'max': 90},
                                  '1': {'min': 0,   'max': 90},
                                  },
                              'LF':
                              {
                                  '5': {'min': 0,   'max': 45},
                                  '4': {'min': -20, 'max': 20},
                                  '3': {'min': -15, 'max': 90},
                                  '2': {'min': 0,   'max': 90},
                                  '1': {'min': 0,   'max': 90},
                                  },
                              'WR':
                              {
                                  '1': {'min': -40, 'max': 28},
                                  '2': {'min': -30, 'max': 10},
                                  },
                              }

        self._clear_j4 = {'FF': {'MF': 'max', 'RF': 'min', 'LF': 'min'},
                          'MF': {'FF': 'min', 'RF': 'min', 'LF': 'min'},
                          'RF': {'FF': 'min', 'MF': 'min', 'LF': 'min'},
                          'LF': {'FF': 'min', 'MF': 'min', 'RF': 'max'}}
        self._joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)
        self._hand_commander = SrHandCommander(name=(side + "_hand"))
        for key, value in self._hand_commander.get_current_state().iteritems():
            requested_joint = key.replace(self._hand_prefix, "")
            self.requested_joints.append(requested_joint)
        self._controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix], [joint.lower() for joint in self.requested_joints])
        self._hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        self._switch_controller_service = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        self._pwm_command_publishers = {}
        self.setup_pwm_publishers()
        self.initialise_output_dictionary()
        self.setup_controller_subscribers()
        # while not rospy.is_shutdown():
        #     self.run()

    def run_2(self):
        CONST_EXIT_CHAR = 'Q'
        CONST_ALL_CHAR = 'A'
        CONST_POSITION_CHAR = 'P'
        CONST_EFFORT_CHAR = 'E'
        in_string = raw_input("Please enter joint ('q' to exit, 'a' for all):").upper()
        input_commnad = in_string.split('_')
        command_type = input_commnad[0].upper()
        joint = input_commnad[1].upper()
        angle = input_commnad[2].lower()
        if 'min' in angle:
            angle = self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['min']
        elif 'max' in angle:
            angle = self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['max']
        rospy.loginfo("type: %s joint: %s angle: %s", command_type, joint, angle)
        if (command_type not in CONST_ALL_CHAR and
            command_type not in CONST_EXIT_CHAR and
            command_type not in CONST_EFFORT_CHAR and
            command_type not in CONST_POSITION_CHAR):
            rospy.logerr("command type %s not recognised", command_type)
            return
        if (joint not in self.requested_joints and
            joint not in 'ALL'):
            rospy.logerr("joint %s not recognised", joint)
            return
        if (float(angle) > self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['max']):
            rospy.logerr("angle outside range for %s: %s > %s", joint, angle, self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['max'])
            return
        if (float(angle) < self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['min']):
            rospy.logerr("angle outside range for %s: %s < %s", joint, angle, self._joint_ranges[joint.split('J')[0]][joint.split('J')[1]]['min'])
            return
        if CONST_EXIT_CHAR in command_type:
            rospy.loginfo("Quitting...")
            sys.exit(0)
        print "type: {type} joint: {joint} angle: {angle}".format(type=command_type, joint=joint, angle=angle)
        rospy.loginfo("type: %s joint: %s angle: %s", command_type, joint, angle)
        

    def run(self):
        CONST_EXIT_CHAR = 'Q'
        CONST_ALL_CHAR = 'A'
        CONST_POSITION_CHAR = 'P'
        CONST_EFFORT_CHAR = 'E'
        in_string = ""
        while in_string not in self.requested_joints:
                print "Available joints: ", self.requested_joints
                in_string = raw_input("Please enter joint ('q' to exit, 'a' for all):").upper()
                if CONST_EXIT_CHAR.lower() == in_string.lower():
                    sys.exit(0)
                if CONST_ALL_CHAR.lower() == in_string.lower():
                    rospy.loginfo("running all")
                    for joint in self.requested_joints:
                        self.run_joint(joint)
                if CONST_POSITION_CHAR.lower() == in_string.lower():
                    rospy.loginfo("switching to position mode")
                    self.switch_to_position()
                if CONST_EFFORT_CHAR.lower() == in_string.lower():
                    rospy.loginfo("switching to effort mode")
                    self.switch_to_effort()
                if (in_string not in self.requested_joints and
                        in_string not in CONST_EXIT_CHAR and
                        in_string not in CONST_POSITION_CHAR and
                        in_string not in CONST_EFFORT_CHAR and
                        in_string not in CONST_ALL_CHAR):
                    rospy.logwarn("%s is not a valid joint or command", in_string)
        self.run_joint(in_string)

    def switch_finger_to_effort(self, finger):
        joints_to_change = []
        for joint in self.requested_joints:
            if finger in joint:
                joints_to_change.append(joint)
        temp_controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix], [joint.lower() for joint in joints_to_change])
        temp_controller_helper.change_hand_ctrl("effort")

    def switch_finger_to_position(self, finger):
        joints_to_change = []
        for joint in self.requested_joints:
            if finger.lower() in joint.lower():
                joints_to_change.append(joint)
        temp_controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix], [joint.lower() for joint in joints_to_change])
        temp_controller_helper.change_hand_ctrl("position")

    def switch_to_effort(self):
        self._controller_helper.change_hand_ctrl("effort")
        try:
            resp1 = self._switch_controller_service([], self._j0_position_controllers, SwitchControllerRequest.BEST_EFFORT, False, 0.0)
        except rospy.ServiceException:
            rospy.logerr("Failed to stop joint zero position controllers")
        self._mode = 'effort'

    def switch_to_position(self):
        self._controller_helper.change_hand_ctrl("position")
        try:
            resp1 = self._switch_controller_service(self._j0_position_controllers, [], SwitchControllerRequest.BEST_EFFORT, False, 0.0)
        except rospy.ServiceException:
            rospy.logerr("Failed to start joint zero position controllers")
        self._mode = 'position'

    def publish_pwm(self, joint, pwm):
        if 'effort' in self._mode:
            rospy.loginfo("Applying a PWM of %s to joint %s", str(pwm), joint)
            self._pwm_command_publishers[joint.upper()].publish(Float64(pwm))
        else:
            rospy.logerr("Mode not set to effort, please change this before applying a PWM")

    def run_joint(self, joint):
        if 'position' not in self._mode:
            self.go_to_zero_joint_state()
            self.test_joint(joint)
            self.go_to_zero_joint_state()
        else:
            rospy.logerr("Mode not set to position, please change this before requesting a joint position")

    def test_joint(self, joint):
        if '4' in joint and 'th' not in joint.lower():
            self.free_j4(joint)
        rospy.loginfo("Testing joint %s:", joint)
        self.activate_output(joint, True)
        self.move_joint_minmax(joint, 'min')
        self.move_joint_minmax(joint, 'max')
        self.move_joint_angle(joint, 0)
        self.activate_output(joint, False)
        self.write_output_dictionaries(joint)
        print
        print

    def free_j4(self, joint):
        print
        rospy.loginfo("Making some space around %s:", joint)
        which_finger = joint.split('J')[0]
        self._clear_j4[which_finger]
        for key, value in self._clear_j4[which_finger].iteritems():
            self.move_joint_minmax(key + 'J4', value, wait=False)
        self.move_joint_angle(joint, 0, wait=True)
        print

    def write_output_dictionaries(self, joint):
        filename = self.construct_filename(joint)
        self.write_output_dictionary('jointstate_',
                                     filename,
                                     self._output_jointstate,
                                     self._output_jointstate_keys)
        self.write_output_dictionary('controllerstate_',
                                     filename,
                                     self._controller_subscribers[joint].output_controller_state,
                                     self._controller_subscribers[joint].output_controller_state_keys)

    def initialise_output_dictionary(self):
        for key in self._output_jointstate_keys:
            self._output_jointstate[key] = []

    def activate_output(self, joint, enable):
        if enable:
            self._controller_subscribers[joint.upper()].initialise_output_dictionary()
            self.initialise_output_dictionary()
            self.active_tests.append(joint)
        else:
            self.active_tests = []
        self._controller_subscribers[joint].enable_output = enable

    def store_joint_state(self, joint):
        idx = self._last_joint_state.name.index(self._hand_prefix + joint)
        position = self._last_joint_state.position[idx]
        velocity = self._last_joint_state.velocity[idx]
        effort = self._last_joint_state.effort[idx]
        timestamp = self._last_joint_state.header.stamp
        self._output_jointstate['timestamp'].append(timestamp)
        self._output_jointstate['position'].append(position)
        self._output_jointstate['velocity'].append(velocity)
        self._output_jointstate['effort'].append(effort)

    def go_to_zero_joint_state(self):
        self._hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)

    def move_joint_minmax(self, joint, min_max='min', wait=True):
        target_joint_states = {}
        joint_number = joint.split('J')[1]
        finger = joint.split('J')[0]
        angle = self._joint_ranges[finger][joint_number][min_max]
        rospy.loginfo("Moving: %s to %s (%s)", joint, min_max, str(angle))
        target_joint_states[self._hand_prefix + joint.upper()] = float(angle)
        self._hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def move_joint_angle(self, joint, angle, wait=True):
        target_joint_states = {}
        target_joint_states[self._hand_prefix + joint.upper()] = angle
        rospy.loginfo("Moving: %s  to: %s", joint, str(angle))
        self._hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def create_controller_subscriber(self, key):
        controller_state_monitor = ControllerStateMonitor(key)
        topic_name = "/sh_" + self._hand_prefix + key.lower() + "_position_controller/state"

        def controller_subscriber(msg):
            if controller_state_monitor.enable_output:
                controller_state_monitor.output_controller_state['command'].append(msg.command)
                controller_state_monitor.output_controller_state['timestamp'].append(msg.header.stamp)
                controller_state_monitor.output_controller_state['setpoint'].append(msg.set_point)
                controller_state_monitor.output_controller_state['process_value'].append(msg.process_value)
                controller_state_monitor.output_controller_state['process_value_dot'].append(msg.process_value_dot)

        controller_state_monitor.subscriber = rospy.Subscriber(topic_name, JointControllerState, controller_subscriber)
        return controller_state_monitor

    def setup_controller_subscribers(self):
        for joint in self.requested_joints:
            self._controller_subscribers[joint] = self.create_controller_subscriber(joint)

    def setup_pwm_publishers(self):
        for joint in self.requested_joints:
            self._pwm_command_publishers[joint] = rospy.Publisher("/sh_%s_effort_controller/command" %	(self._hand_prefix + joint.lower()), Float64, queue_size=2)

    def joint_state_cb(self, msg):
        self._joint_state_updated = True
        self._last_joint_state.header = msg.header
        self._last_joint_state.name = msg.name
        self._last_joint_state.position = msg.position
        self._last_joint_state.velocity = msg.velocity
        self._last_joint_state.effort = msg.effort
        if self.active_tests:
            self.store_joint_state(self.active_tests[0])

    def construct_filename(self, joint):
        day = datetime.datetime.now().day
        month = datetime.datetime.now().month
        year = datetime.datetime.now().year
        hour = datetime.datetime.now().hour
        minute = datetime.datetime.now().minute
        second = datetime.datetime.now().second
        filename = joint + '_' + str(year) + '_' + str(month) + '_' +\
            str(day) + '_' + str(hour) + '_' + str(minute) +\
            '_' + str(second) + '.csv'
        return filename

    def write_output_dictionary(self, filename_prefix, filename, output_dictionary, dictionary_keys):
        fieldnames = []
        rows = []
        output_filename = filename_prefix + filename
        for key in dictionary_keys:
            fieldnames.append(key)
        length_of_list = len(output_dictionary[dictionary_keys[0]])
        for i in range(0, length_of_list):
            line = {}
            for key in dictionary_keys:
                line[key] = output_dictionary[key][i]
            rows.append(line)
        self.write_csv(output_filename, fieldnames, rows)

    def write_csv(self, filename, fieldnames, rows):
        with open(filename, mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
        rospy.loginfo("file: %s saved", filename)


if __name__ == '__main__':
    rospy.init_node("sr_test_force_resolution")
    tfr = TestForceResolution()
    # test_force_resolution = TestForceResolution()
