#!/usr/bin/python

import rospy
import copy
from control_msgs.msg import JointControllerState
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
        self.new_data = False
        self.command = 0
        self.sub = []

    def unsubscribe_all(self):
        for subscriber in self.sub:
            subscriber.unsubscribe()


class TestForceResolution():
    def __init__(self):
        self.requested_joints = ['FFJ3', 'FFJ4', 'MFJ3', 'MFJ4', 'RFJ3', 'RFJ4', 'LFJ3', 'LFJ4']
        self._joint_states_zero = {'rh_FFJ1': 0, 'rh_FFJ2': 0, 'rh_FFJ3': 0, 'rh_FFJ4': 0,
                         'rh_MFJ1': 0, 'rh_MFJ2': 0, 'rh_MFJ3': 0, 'rh_MFJ4': 0,
                         'rh_RFJ1': 0, 'rh_RFJ2': 0, 'rh_RFJ3': 0, 'rh_RFJ4': 0,
                         'rh_LFJ1': 0, 'rh_LFJ2': 0, 'rh_LFJ3': 0, 'rh_LFJ4': 0, 'rh_LFJ5': 0,
                         'rh_THJ1': 0, 'rh_THJ2': 0, 'rh_THJ3': 0, 'rh_THJ4': 0, 'rh_THJ5': 0,
                         'rh_WRJ1': 0, 'rh_WRJ2': 0}
        self._controller_subscribers = {}
        self._joint_ranges = {'3': {'min': 0, 'max': 90},
                              '4': {'min': -20, 'max': 20}}
        self.hand_commander = SrHandCommander(name="right_hand")
        self.hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        self.setup_controller_subscribers()
        self.perform_tests()

    def perform_tests(self):
        for joint in self.requested_joints:
            self.test_joint(joint)

    def test_joint(self, joint):
        self.move_joint(joint, 'min')
        self.move_joint(joint, 'max')
        self.go_to_zero_joint_state()

    def go_to_zero_joint_state(self):
        self.hand_commander.move_to_joint_value_target(self._joint_states_zero, wait=True, angle_degrees=True)
        
    def move_joint(self, joint, min_max='min'):
        target_joint_states = {}
        target_joint_states["rh_" + joint.upper()] = self._joint_ranges[joint.split('J')[1]][min_max]
        print target_joint_states
        self.hand_commander.move_to_joint_value_target(target_joint_states, wait=True, angle_degrees=True)

    def create_controller_subscriber(self, key):
        controller_state_monitor = ControllerStateMonitor(key)
        topic_name = "/sh_rh_" + key.lower() + "_position_controller/state"

        def controller_subscriber(msg):
            controller_state_monitor.command = msg.command
            controller_state_monitor.new_data = True
            # rospy.loginfo("%s: %f", controller_state_monitor.name, controller_state_monitor.command)

        controller_state_monitor.sub.append(rospy.Subscriber(topic_name, JointControllerState, controller_subscriber))
        return controller_state_monitor

    def setup_controller_subscribers(self):
        for joint in self.requested_joints:
            self._controller_subscribers[joint] = self.create_controller_subscriber(joint)


if __name__ == '__main__':
    rospy.init_node("sr_test_force_resolution")
    test_force_resolution = TestForceResolution()
