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


class TestHandCommand():
    def __init__(self, joint_ranges, requested_joints):
        self.requested_joints = requested_joints
	self._joint_ranges = joint_ranges
        self.cmd = None
        self.joint = None
	self.finger = None
        self.value = None
        self.finger_joint = None
        self.joints_list = []
        self.all = False
        self.CONST_EXIT_CHAR = 'Q'
        self.CONST_POSITION_CHAR = 'P'
        self.CONST_EFFORT_CHAR = 'E'
        self.allowed_commands = [self.CONST_EXIT_CHAR, self.CONST_POSITION_CHAR, self.CONST_EFFORT_CHAR]

    def validate(self):
        if self.cmd not in self.allowed_commands:
            rospy.logwarn("Invalid command character: %s", str(self.cmd))
            return False

    def quit(self):
        if self.CONST_EXIT_CHAR in self.cmd:
            return True
        return False

    def position_mode(self):
        if self.CONST_POSITION_CHAR in self.cmd:
            return True
        return False

    def effort_mode(self):
        if self.CONST_EFFORT_CHAR in self.cmd:
            return True
        return False
		
    def parse(self, in_string):
        if in_string == "":
            return False
        if self.CONST_EXIT_CHAR in in_string[0].upper():
            self.cmd = 'Q'
            return True
        split_command = in_string.split('_')
        if len(split_command) < 3:
            rospy.logwarn("Invalid command: %s", str(in_string))
            return False
        self.cmd = split_command[0].upper()
        if self.cmd not in self.allowed_commands:
            rospy.logwarn("Invalid command character: %s", str(self.cmd))
            return False
	joint = split_command[1].upper()
	if joint.split('J')[0] == '':
            self.all = True
            self.finger = 'ALL'
	else:
            self.all = False
            self.finger = joint.split('J')[0].upper()
        self.joint = joint.split('J')[1]
        for t in split_command:
            rospy.logwarn("t: %s", t)
        if not self.all:
            converted_value = self.convert_min_max(split_command[2])
            if not converted_value:
                return False
            else:
                self.value = converted_value
            if self.CONST_POSITION_CHAR in self.cmd:
                maximum = self._joint_ranges[self.finger][self.joint]['max']
                minimun = self._joint_ranges[self.finger][self.joint]['min']
                if float(self.value) > maximum:
                    rospy.logerr("angle outside range for %s: %s > %s", self.joint, self.value, maximum)
                    return False
                if float(self.value) < minimun:
                    rospy.logerr("angle outside range for %s: %s < %s", self.joint, self.value, minimun)
                    return False
            if self.finger + 'J' + self.joint not in self.requested_joints:
                rospy.logerr("joint %s not recognised", self.finger + 'J' + self.joint)
                for t in self.requested_joints:
                    print "r: {}".format(t)
                return False
                self.finger_joint = self.finger + 'J' + self.joint
        else:
            if self.is_numeric(split_command[2]):
                self.value = split_command[2]
            else:
                return False
            self.joints_list = [x for x in self.requested_joints if 'J' + self.joint in x and 'TH' not in x]
        return True

    def convert_min_max(self, in_value):
        if self.is_numeric(in_value):
            return in_value
        if 'min' in in_value.lower():
            out_value = self._joint_ranges[self.finger][self.joint]['min']
            rospy.loginfo("joint %s to %s is %s degrees", self.joint, 'min', str(out_value))
        elif 'max' in in_value.lower():
            out_value = self._joint_ranges[self.finger][self.joint]['max']
            rospy.loginfo("joint %s to %s is %s degrees", self.joint, 'max', str(out_value))
        else:
            rospy.logwarn("{VALUE} is not numeric, or 'min' or 'max'. Please try again")
            return False
        return out_value

    def is_numeric(self, value):
        unicode_value = unicode(value)
        if unicode_value.count('-') > 1 or unicode_value.count('.') > 1:
            return False
        return unicode_value.replace('-', '').replace('.', '').isnumeric()

    def reset(self):
        self.cmd = None
        self.joint = None
	self.finger = None
        self.value = None
        self.all = False
        self.finger_joint = None
        self.joint_list = []


class TestForceResolution():
    def __init__(self, side="right"):
        self.active_tests = []
        self._controller_subscribers = {}
        self._last_joint_state = JointState()
        self._output_jointstate = {}
        self._hand_prefix = side[0] + 'h_'
        self._output_jointstate_keys = ['timestamp', 'position', 'velocity', 'effort']
        self._fingers_with_j0 = ['ff', 'mf', 'rf', 'lf']
        self._mode = ''
        self._CONST_EXIT_CHAR = 'Q'
        self._CONST_POSITION_CHAR = 'P'
        self._CONST_EFFORT_CHAR = 'E'
        self._j0_position_controllers = ["sh_{0}{1}j0_position_controller".format(self._hand_prefix, joint)
                                         for joint in self._fingers_with_j0]
        self.requested_joints = []
        joint_states_zero = {'FFJ1': 0, 'FFJ2': 0, 'FFJ3': 0, 'FFJ4': 0,
                                   'MFJ1': 0, 'MFJ2': 0, 'MFJ3': 0, 'MFJ4': 0,
                                   'RFJ1': 0, 'RFJ2': 0, 'RFJ3': 0, 'RFJ4': 0,
                                   'LFJ1': 0, 'LFJ2': 0, 'LFJ3': 0, 'LFJ4': 0, 'LFJ5': 0,
                                   'THJ1': 0, 'THJ2': 0, 'THJ3': 0, 'THJ4': 0, 'THJ5': 0,
                                   'WRJ1': 0, 'WRJ2': 0}
        self._joint_states_zero = {}
        for key, value in joint_states_zero.iteritems():
            self._joint_states_zero[self._hand_prefix + key] = value
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
        self._controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix],
                                                   [joint.lower() for joint in self.requested_joints])
        self._hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True):
        self._switch_controller_service = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        self._pwm_command_publishers = {}
        self.setup_pwm_publishers()
        self.initialise_output_dictionary()
        self.setup_controller_subscribers()
	self.command = TestHandCommand(self._joint_ranges, self.requested_joints)
        while not rospy.is_shutdown():
            self.run()
            self.command.reset()

    def run(self):
        self.print_help()
        in_string = raw_input("Please enter command:").upper()
	if not self.command.parse(in_string):
            return
	if self.command.quit():
            rospy.loginfo("Quitting...")
            sys.exit(0)
        rospy.loginfo("type: %s joint: %s finger: %s angle: %s", self.command.cmd, self.command.joint, self.command.finger, self.command.joint)
        rospy.loginfo("\n\n\n\n\n")
        if not self.command.all:
            if self.command.position_mode():
                self.test_joint(self.command.finger_joint, mode='position', value=self.command.value)
            if self.command.effort_mode():
                file_prefix = raw_input("Name/describe the test (adds text to filename, optional)")
                for i in range(0, 5):
                    self.test_joint(self.command.finger_joint, mode='PWM', value=str(float(self.command.value)), prefix=file_prefix + '_plus')
                    rospy.sleep(1)
                    self.test_joint(self.command.finger_joint, mode='PWM', value=str(float(self.command.value)*-1.0), prefix=file_prefix + '_minus')
                    rospy.sleep(1)
        else:
            for joint in self.command.joints_list:
                rospy.logwarn("acting on: %s", joint)
            if self.command.position_mode():
                file_prefix = raw_input("enter file prefix:")
                for joint in self.command.joints_list:
                    if file_prefix == '':
                        file_prefix = None
                    self.test_joint(joint, mode='testing', prefix=file_prefix)
            elif self.command.effort_mode():
                file_prefix = raw_input("enter file prefix:")
                for joint in self.command.joints_list:
                    if 'LF' in joint.upper() or 'RF' in joint.upper():
                        self.command.value = str(float(self.command.value)*(-1.0))
                    self.test_joint(joint, 'PWM', value=self.command.value, prefix=file_prefix + '_plus')
                    rospy.sleep(1)
                    self.test_joint(joint, 'PWM', value=str(float(self.command.value)*(-1.0)), prefix=file_prefix + '_minus')
                    rospy.sleep(1)

    def switch_finger_to_effort(self, finger):
        joints_to_change = []
        for joint in self.requested_joints:
            if finger in joint:
                joints_to_change.append(joint)
        temp_controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix],
                                                  [joint.lower() for joint in joints_to_change])
        temp_controller_helper.change_hand_ctrl("effort")

    def switch_finger_to_position(self, finger):
        joints_to_change = []
        for joint in self.requested_joints:
            if finger.lower() in joint.lower():
                joints_to_change.append(joint)
        temp_controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix],
                                                  [joint.lower() for joint in joints_to_change])
        temp_controller_helper.change_hand_ctrl("position")

    def switch_to_effort(self):
        self._controller_helper.change_hand_ctrl("effort")
        self._mode = 'effort'

    def switch_to_position(self):
        self._controller_helper.change_hand_ctrl("position")
        self._mode = 'position'

    def publish_pwm(self, joint, pwm):
        if 'effort' in self._mode:
            rospy.loginfo("Applying a PWM of %s to joint %s", str(pwm), joint)
            self._pwm_command_publishers[joint.upper()].publish(Float64(float(pwm)))
        else:
            rospy.logerr("Mode not set to effort, please change this before applying a PWM")

    def test_joint(self, joint, mode='testing', value=0, sleep=3, prefix=''):
        print joint, joint.lower(), value
        if '4' in joint and 'th' not in joint.lower():
            self.free_j4(joint)
            rospy.sleep(1.5)
        file_prefix = prefix
        if prefix == '':
            file_prefix = raw_input("Name/describe the test (adds text to filename, optional)")
        if prefix == None:
            file_prefix = ''
        rospy.loginfo("Testing joint %s:", joint)
        self.activate_output(joint, True)
        if mode == 'testing':
            self.move_joint_minmax(joint, 'min')
            self.move_joint_minmax(joint, 'max')
            self.move_joint_angle(joint, 0)
        elif mode == 'PWM':
            rospy.logwarn("val: %s", str(value))
            self.move_joint_pwm(joint, value)
            rospy.sleep(sleep)
        elif mode == 'position':
            self.move_joint_angle(joint, float(value), wait=True)
        self.activate_output(joint, False)
        self.write_output_dictionaries(joint, file_prefix)
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

    def write_output_dictionaries(self, joint, prefix=''):
        filename = self.construct_filename(joint, prefix)
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

    def move_joint_pwm(self, joint, pwm):
        self.switch_to_effort()
        self.publish_pwm(joint, pwm)

    def move_joint_minmax(self, joint, min_max='min', wait=True):
        self.switch_to_position()
        target_joint_states = {}
        joint_number = joint.split('J')[1]
        finger = joint.split('J')[0]
        angle = self._joint_ranges[finger][joint_number][min_max]
        rospy.loginfo("Moving: %s to %s (%s)", joint, min_max, str(angle))
        target_joint_states[self._hand_prefix + joint.upper()] = float(angle)
        self._hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def move_joint_angle(self, joint, angle, wait=True):
        self.switch_to_position()
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

        controller_state_monitor.subscriber = rospy.Subscriber(topic_name,
                                                               JointControllerState,
                                                               controller_subscriber)
        return controller_state_monitor

    def setup_controller_subscribers(self):
        for joint in self.requested_joints:
            self._controller_subscribers[joint] = self.create_controller_subscriber(joint)

    def setup_pwm_publishers(self):
        for joint in self.requested_joints:
            self._pwm_command_publishers[joint] = rospy.Publisher("/sh_%s_effort_controller/command" %
                                                                  (self._hand_prefix + joint.lower()),
                                                                  Float64, queue_size=2)

    def joint_state_cb(self, msg):
        self._last_joint_state.header = msg.header
        self._last_joint_state.name = msg.name
        self._last_joint_state.position = msg.position
        self._last_joint_state.velocity = msg.velocity
        self._last_joint_state.effort = msg.effort
        if self.active_tests:
            self.store_joint_state(self.active_tests[0])

    def construct_filename(self, joint, prefix_s=''):
        day = datetime.datetime.now().day
        month = datetime.datetime.now().month
        year = datetime.datetime.now().year
        hour = datetime.datetime.now().hour
        minute = datetime.datetime.now().minute
        second = datetime.datetime.now().second
        prefix = self._mode
        if prefix == 'effort':
            prefix = 'direct_PWM_'
        else:
            prefix = prefix + '_'
        filename = prefix + joint + '_' + str(year) + '_' + str(month) + '_' +\
            str(day) + '_' + str(hour) + '_' + str(minute) +\
            '_' + str(second) + '_' + prefix_s + '.csv'
        return filename

    def print_help(self):
        print ""
        print "Command sytax:"
        print "  {COMMAND_CHAR}_{JOINT_NAME}_{ANGLE/PWM/LIMIT}"
        print ""
        print "where:"
        print "  {COMMAND_CHAR} can be:"
        print "    p - position mode (move to ANGLE (e.g. 60) or LIMIT (e.g. min or max)"
        print "    e - effort / direct PWM mode (apply fixed PWM to joint)"
        print "    q - Quit / exit"
        print ""
        print "  {JOINT_NAME} can be either:"
        print "    {FINGER_NAME}J{JOINT_NUMBER} - for testing individual joints (e.g. FFJ3, RFJ4 etc...)"
        print "  OR:"
        print "    J{JOINT_NUMBER}              - for testing the same joint on all fingers (e.g. J3, J4)"
        print ""
        print "                                 - If COMMAND_CHAR is 'p', using J{JOINT_NUMBER} will"
        print "                                   move all joints in sequence to min, then max, then zero"
        print ""
        print "                                 - If COMMAND_CHAR is 'e', using J{JOINT_NUMBER} will"
        print "                                   move apply PWM for three seconds, then -PWM for three seconds"
        print ""
        print "  {ANGLE/PWM/LIMIT} can be:"
        print "    when COMMAND_CHAR is 'p' - this value is taken as an angle (if a number is specified)"
        print "                             - or as a command to move to joint limit (e.g. 'min', 'max')"
        print ""
        print "    when COMMAND_CHAR is 'e' - This value is taken as a direct PWM command"
        print ""
        print "Examples:"
        print "  e_ffj3_200 - Apply a constant PWM of 200 to joint FFJ3"
        print "  e_ffj3_-240 - Apply a constant PWM of -240 to joint FFJ3"
        print "  p_ffj3_60  - Move ffj3 to 60 degrees"
        print "  p_ffj3_-5  - Move ffj3 to -5 degrees"
        print "  p_j3_0     - Test all joint 3's  (position mode)  in fingers: FF, RF, MF, LF"
        print "  e_j3_150   - Test all joint 3's (direct PWM mode) in fingers: FF, RF, MF, LF"
        print "  p_ffj4_min - Move FFJ4 to it's minimum angle"
        print "  p_rfj3_max - Move RFJ3 to it's maximum angle"
        print ""
        print "(Note: Using 'p' or 'e' in COMMAND_CHAR will change ALL controllers to "
        print "position or effort, not just the controllers for the finger specified)"
        print ""

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
