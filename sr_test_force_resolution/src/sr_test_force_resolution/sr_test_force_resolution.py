#!/usr/bin/python

import rospy
import copy
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from sr_robot_commander.sr_hand_commander import SrHandCommander


'''
Position sensors
strain gauge sensors
current sensors
PWM going into each motor

these 4 sets of data for the following joints
    FF3, FF4
    MF3, MF4
    RF3, RF4
    LF3, LF4
'''


class ControllerStateMonitor():
    def __init__(self, name):
        self.name = name
        self.enable_output = False
        self.new_data = False
        self.command = 0
        self.set_point = 0
        self.process_value = 0
        self.process_value_dot = 0
        self.sub = []

    def unsubscribe_all(self):
        for subscriber in self.sub:
            subscriber.unsubscribe()


class TestForceResolution():
    def __init__(self):
        self.active_tests = []
        self.requested_joints = ['FFJ3', 'FFJ4', 'MFJ3', 'MFJ4', 'RFJ3', 'RFJ4', 'LFJ3', 'LFJ4']
        self._joint_states_zero = {'rh_FFJ1': 0, 'rh_FFJ2': 0, 'rh_FFJ3': 0, 'rh_FFJ4': 0,
                         'rh_MFJ1': 0, 'rh_MFJ2': 0, 'rh_MFJ3': 0, 'rh_MFJ4': 0,
                         'rh_RFJ1': 0, 'rh_RFJ2': 0, 'rh_RFJ3': 0, 'rh_RFJ4': 0,
                         'rh_LFJ1': 0, 'rh_LFJ2': 0, 'rh_LFJ3': 0, 'rh_LFJ4': 0, 'rh_LFJ5': 0,
                         'rh_THJ1': 0, 'rh_THJ2': 0, 'rh_THJ3': 0, 'rh_THJ4': 0, 'rh_THJ5': 0,
                         'rh_WRJ1': 0, 'rh_WRJ2': 0}
        self._joint_ranges = {'4': 
                                 {
                                  'FF': {'min': -20, 'max': 20},
                                  'MF': {'min': -20, 'max': 20},
                                  'RF': {'min': -20, 'max': 20},
                                  'LF': {'min': -20, 'max': 20},
                                  }, 
                               '3':
                                 {
                                  'FF': {'min': 0, 'max': 90},
                                  'MF': {'min': 0, 'max': 90},
                                  'RF': {'min': 0, 'max': 90},
                                  'LF': {'min': 0, 'max': 90},
                                  },
                              }
        self._m_joint_ranges = {'4': {'min': -20, 'max': 20},
                                '3': {'min':   0, 'max': 90}}

        self._clear_j4 = { 'FF': {'MF': 'max', 'RF': 'min', 'LF': 'min'},
                           'MF': {'FF': 'min', 'RF': 'min', 'LF': 'min'},
                           'RF': {'FF': 'min', 'MF': 'min', 'LF': 'min'},
                           'LF': {'FF': 'min', 'MF': 'min', 'RF': 'max'}}
        self._controller_subscribers = {}
        self._last_joint_state = JointState()
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)
        self.hand_commander = SrHandCommander(name="right_hand")
        self.hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        self.setup_controller_subscribers()
        self.perform_tests()

    def perform_tests(self):
        self.go_to_zero_joint_state()
        for joint in self.requested_joints:
            self.test_joint(joint)
            self.go_to_zero_joint_state()

    def free_j4(self, joint):
        which_finger = joint.split('J')[0]
        self._clear_j4[which_finger]
        for key, value in self._clear_j4[which_finger].iteritems():
            rospy.loginfo("moving %s to %s", key + 'J4', value)
            self.move_joint_minmax(key + 'J4', value, wait=False)
        self.move_joint_angle(joint, 0, wait=True)


    def all_minmax(self, minmax='min', wait=False):
        for joint in self.requested_joints:
            self.move_joint_minmax(joint, minmax, wait)

    def test_joint(self, joint):
        if '4' in joint:
            self.free_j4(joint)
        self.activate_output(joint, True)
        self.move_joint_minmax(joint, 'min')
        self.move_joint_minmax(joint, 'max')
        self.move_joint_angle(joint, 0)
        self.activate_output(joint, False)

    def activate_output(self, joint, enable):
        self._controller_subscribers[joint].enable_output = enable
        if enable:
            self.active_tests.append(joint)
        else:
            self.active_tests = []

    def print_joint_state(self, joint):
        idx = self._last_joint_state.name.index("rh_" + joint)
        position = self._last_joint_state.position[idx]
        velocity = self._last_joint_state.velocity[idx]
        effort = self._last_joint_state.effort[idx]
        rospy.loginfo("pos: %f vel: %f eff: %f", position, velocity, effort)

    def go_to_zero_joint_state(self):
        self.hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        
    def move_joint_minmax(self, joint, min_max='min', wait=True):
        target_joint_states = {}
        joint_number = joint.split('J')[1]
        # which_finger = joint.split('J')[0]
        # rospy.loginfo("which_finger: %s  joint_number: %s", which_finger, joint_number)
        # target_joint_states["rh_" + joint.upper()] = self._m_joint_ranges[joint_number][which_finger][min_max]
        target_joint_states["rh_" + joint.upper()] = self._m_joint_ranges[joint_number][min_max]
        print target_joint_states
        self.hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def move_joint_angle(self, joint, angle, wait=True):
        target_joint_states = {}
        target_joint_states["rh_" + joint.upper()] = angle
        self.hand_commander.move_to_joint_value_target(target_joint_states, wait=wait, angle_degrees=True)

    def create_controller_subscriber(self, key):
        controller_state_monitor = ControllerStateMonitor(key)
        topic_name = "/sh_rh_" + key.lower() + "_position_controller/state"

        def controller_subscriber(msg):
            controller_state_monitor.command = msg.command
            controller_state_monitor.set_point = msg.set_point
            controller_state_monitor.process_value = msg.process_value
            controller_state_monitor.process_value_dot = msg.process_value_dot
            controller_state_monitor.new_data = True
            if controller_state_monitor.enable_output:
                rospy.loginfo("name   command  setpoint  process  proc_dot")
                rospy.loginfo("%s: %f %f %f %f", controller_state_monitor.name, controller_state_monitor.command, controller_state_monitor.set_point, controller_state_monitor.process_value, controller_state_monitor.process_value_dot)

        controller_state_monitor.sub.append(rospy.Subscriber(topic_name, JointControllerState, controller_subscriber))
        return controller_state_monitor

    def setup_controller_subscribers(self):
        for joint in self.requested_joints:
            self._controller_subscribers[joint] = self.create_controller_subscriber(joint)

    def joint_state_cb(self, msg):
        self._joint_state_updated = True
        self._last_joint_state.header = msg.header
        self._last_joint_state.name = msg.name
        self._last_joint_state.position = msg.position
        self._last_joint_state.velocity = msg.velocity
        self._last_joint_state.effort = msg.effort
        if self.active_tests:
            self.print_joint_state(self.active_tests[0])


if __name__ == '__main__':
    rospy.init_node("sr_test_force_resolution")
    tfr = TestForceResolution()
    # tfr.free_j4('FFJ4')
