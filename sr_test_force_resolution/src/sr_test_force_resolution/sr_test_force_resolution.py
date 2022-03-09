#!/usr/bin/python3
# Copyright 2021, 2022 Shadow Robot Company Ltd.
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
import rospkg
import yaml
import sys
import csv
import datetime
import qprompt
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_controllers_tools.sr_controller_helper import ControllerHelper
from controller_manager_msgs.srv import SwitchController
from builtins import str

class ControllerStateMonitor():
    def __init__(self, name):
        self.name = name
        self.enable_output = False
        self.subscriber = None
        self._CONST_CONTROLLER_STATE_KEYS = ['timestamp', 'command', 'setpoint', 'process_value', 'process_value_dot']
        self.output_controller_state = {}
        self.initialise_output_dictionary()

    def initialise_output_dictionary(self):
        for key in self._CONST_CONTROLLER_STATE_KEYS:
            self.output_controller_state[key] = []

    def unsubscribe_all(self):
        self.subscriber.unsubscribe()


class TestHandCommand():
    def __init__(self, joint_ranges, requested_joints):
        self.requested_joints = requested_joints
        self._joint_ranges = joint_ranges
        self.joint = None
        self.finger = None
        self.value = None
        self.finger_joint = None
        self.joints_list = []
        self.all_fingers = False
        self.control_type = ""
        self._menu_lv1 = qprompt.Menu()
        self._setup_menus()

    def _setup_menus(self):
        self._menu_lv1.add("p", "position", self.set_control_type_position)
        self._menu_lv1.add("e", "effort", self.set_control_type_effort)
        self._menu_lv1.add("q", "quit", self.quit)

    def set_joint(self):
        pass

    def set_control_type_position(self):
        self.control_type = "position"

    def set_control_type_effort(self):
        self.control_type = "effort"

    def quit(self):
        rospy.loginfo("Quitting...")
        sys.exit(0)

    def menu_level_1(self):
        self.control_type = ""
        while self.control_type == "":
            self._menu_lv1.show()
        self.menu_level_2()

    def menu_level_2(self):
        joint_valid = False
        self.joint = None
        while not joint_valid:
            qprompt.info("{} selected!".format(self.control_type))
            prompt_string = "Please enter a joint (e.g. FFJ3), or to"\
                            "select the same joint across multiple fingers ommit the finger prefix (e.g. J3)"
            joint_string = qprompt.ask_str(prompt_string).upper()
            joint_valid = self.validate_joint(joint_string)
        self.finger_joint = joint_string
        if 'J' != joint_string[0]:
            self.all_fingers = False
            self.joint = joint_string.split('J')[1]
            self.finger = joint_string.split('J')[0]
            self.menu_level_3()
        else:
            self.all_fingers = True
            self.joint = joint_string[1]
            self.joints_list = [x for x in self.requested_joints if 'J' + self.joint in x and 'TH' not in x]
            if self.control_type == 'effort':
                self.menu_level_3()
            return
        rospy.logwarn("%s %s", str(self.finger_joint), str(self.joint))

    def menu_level_3(self):
        value_valid = False
        self.value = None
        prompt_string = "Please enter a value"
        if self.control_type != 'effort':
            prompt_string = prompt_string + " (in degrees), or 'min'/'max'"
        else:
            prompt_string = prompt_string + " (PWM)"
        while not value_valid:
            l = qprompt.ask(prompt_string)
            value_valid = self.validate_value(l)
        self.value = self.convert_min_max(l)

    def validate_value(self, value):
        if 'min' == value or 'max' == value:
            return True
        if 'position' == self.control_type:
            if not self.check_min_max_limits(value):
                return False
            return True
        return True

    def validate_joint(self, joint):
        if joint[0] == 'J':
            try:
                int(joint[1])
            except ValueError:
                return False
            else:
                return True
        for j in self.requested_joints:
            if joint.split('J')[0] == j.split('J')[0]:
                for q in self.requested_joints:
                    if joint.split('J')[1] == j.split('J')[1]:
                        return True
        rospy.logwarn("%s is not a valid finger", joint)
        return False

    def check_min_max_limits(self, in_value):
        maximum = self._joint_ranges[self.finger][self.joint]['max']
        minimum = self._joint_ranges[self.finger][self.joint]['min']
        if float(in_value) > maximum:
            rospy.logerr("angle outside range for %s: %s > %s", self.finger_joint, in_value, maximum)
            return False
        if float(in_value) < minimum:
            rospy.logerr("angle outside range for %s: %s < %s", self.finger_joint, in_value, minimum)
            return False
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
        unicode_value = str(value)
        if unicode_value.count('-') > 1 or unicode_value.count('.') > 1:
            return False
        return unicode_value.replace('-', '').replace('.', '').isnumeric()

    def reset(self):
        self.joint = None
        self.finger = None
        self.value = None
        self.all_fingers = False
        self.finger_joint = None
        self.joints_list = []
        self.control_type = ""


class TestForceResolution():
    def __init__(self, side="right"):
        self._active_tests = []
        self._controller_subscribers = {}
        self._last_joint_state = JointState()
        self._output_jointstate = {}
        self._hand_prefix = side[0] + 'h_'
        self._CONST_JOINTSTATE_KEYS = ['timestamp', 'position', 'velocity', 'effort']
        self._fingers_with_j0 = ['ff', 'mf', 'rf', 'lf']
        self._mode = ''
        self._j0_position_controllers = ["sh_{0}{1}j0_position_controller".format(self._hand_prefix, joint)
                                         for joint in self._fingers_with_j0]
        self.requested_joints = []
        rospack = rospkg.RosPack()
        root_path = rospack.get_path('sr_test_force_resolution') + '/config/'
        joint_ranges_yaml_path = root_path + 'joint_ranges.yaml'
        joint_zeros_yaml_path = root_path + 'joint_zeros.yaml'
        clear_j4_signs_path = root_path + 'clear_j4_signs.yaml'
        self._joint_ranges = self.load_yaml(joint_ranges_yaml_path)
        self._joint_states_zero = {}
        joint_states_zero = self.load_yaml(joint_zeros_yaml_path)
        for key, value in joint_states_zero.items():
            self._joint_states_zero[self._hand_prefix + key] = value
        self._clear_j4 = self.load_yaml(clear_j4_signs_path)
        self._joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)
        self._hand_commander = SrHandCommander(name=(side + "_hand"))
        for key, value in self._hand_commander.get_current_state().items():
            requested_joint = key.replace(self._hand_prefix, "")
            self.requested_joints.append(requested_joint)
        self._controller_helper = ControllerHelper([self._hand_prefix[0] + 'h'], [self._hand_prefix],
                                                   [joint.lower() for joint in self.requested_joints])
        self._hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        self._switch_controller_service = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        self._pwm_command_publishers = {}
        self.setup_pwm_publishers()
        self.initialise_output_dictionary()
        self.setup_controller_subscribers()
        self.command = TestHandCommand(self._joint_ranges, self.requested_joints)

    def run(self):
        self.command.reset()
        self.command.menu_level_1()
        file_prefix = qprompt.ask_str("Enter name of test (optional, press enter to skip)")
        if not self.command.all_fingers:
            if self.command.control_type == 'position':
                self.test_joint(self.command.finger_joint, mode='position', value=self.command.value)
            else:
                self.test_joint(self.command.finger_joint, mode='PWM', value=str(float(self.command.value)),
                                prefix=file_prefix + '_plus')
                rospy.sleep(1)
                self.test_joint(self.command.finger_joint, mode='PWM', value=str(float(self.command.value)*-1.0),
                                prefix=file_prefix + '_minus')
                rospy.sleep(1)
                self.move_joint_pwm(self.command.finger_joint, 0)
                
        else:
            for joint in self.command.joints_list:
                rospy.logwarn("acting on: %s", joint)
            if self.command.control_type == 'position':
                for joint in self.command.joints_list:
                    self.test_joint(joint, mode='testing', prefix=file_prefix)
            else:
                for joint in self.command.joints_list:
                    if 'LF' in joint.upper() or 'RF' in joint.upper():
                        self.command.value = str(float(self.command.value)*(-1.0))
                    self.test_joint(joint, 'PWM', value=self.command.value, prefix=file_prefix + '_plus')
                    rospy.sleep(1)
                    self.test_joint(joint, 'PWM', value=str(float(self.command.value)*(-1.0)),
                                    prefix=file_prefix + '_minus')
                    rospy.sleep(1)
                    self.move_joint_pwm(joint, 0)

    def load_yaml(self, in_file):
        try:
            stream = open(in_file, 'r')
            return yaml.load(stream)
        except:
            rospy.logerr("Error loading config file: %s", in_file)
            raise

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

    def switch_all_controllers(self, controller_type):
        self._controller_helper.change_hand_ctrl(controller_type)
        self._mode = controller_type

    def publish_pwm(self, joint, pwm):
        if 'effort' in self._mode:
            rospy.loginfo("Applying a PWM of %s to joint %s", str(pwm), joint)
            self._pwm_command_publishers[joint.upper()].publish(Float64(float(pwm)))
        else:
            rospy.logerr("Mode not set to effort, please change this before applying a PWM")

    def test_joint(self, joint, mode='testing', value=0, sleep=3, prefix=''):
        if '4' in joint and 'th' not in joint.lower():
            self.free_j4(joint)
            rospy.sleep(1.5)
        file_prefix = prefix
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

    def free_j4(self, joint):
        rospy.loginfo("Making some space around %s:", joint)
        which_finger = joint.split('J')[0]
        self._clear_j4[which_finger]
        for key, value in self._clear_j4[which_finger].items():
            self.move_joint_minmax(key + 'J4', value, wait=False)
        self.move_joint_angle(joint, 0, wait=True)

    def write_output_dictionaries(self, joint, prefix=''):
        filename = self.construct_filename(joint, prefix)
        self.write_output_dictionary('jointstate_',
                                     filename,
                                     self._output_jointstate,
                                     self._CONST_JOINTSTATE_KEYS)
        self.write_output_dictionary('controllerstate_',
                                     filename,
                                     self._controller_subscribers[joint].output_controller_state,
                                     self._controller_subscribers[joint]._CONST_CONTROLLER_STATE_KEYS)

    def initialise_output_dictionary(self):
        for key in self._CONST_JOINTSTATE_KEYS:
            self._output_jointstate[key] = []

    def activate_output(self, joint, enable):
        if enable:
            self._controller_subscribers[joint.upper()].initialise_output_dictionary()
            self.initialise_output_dictionary()
            self._active_tests.append(joint)
        else:
            self._active_tests = []
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
        self.switch_all_controllers('effort')
        self.publish_pwm(joint, pwm)

    def move_joint_minmax(self, joint, min_max='min', wait=True):
        self.switch_all_controllers('position')
        target_joint_states = {}
        joint_number = joint.split('J')[1]
        finger = joint.split('J')[0]
        angle = self._joint_ranges[finger][joint_number][min_max]
        rospy.loginfo("Moving: %s to %s (%s)", joint, min_max, str(angle))
        target_joint_states[self._hand_prefix + joint.upper()] = float(angle)
        self._hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def move_joint_angle(self, joint, angle, wait=True):
        self.switch_all_controllers('position')
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
        if self._active_tests:
            self.store_joint_state(self._active_tests[0])

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
