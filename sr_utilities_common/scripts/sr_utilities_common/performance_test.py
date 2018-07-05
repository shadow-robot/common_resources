#!/usr/bin/python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import argparse
from sr_utilities.hand_finder import HandFinder
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug


class PerformanceTest(object):
    def __init__(self):
        rospy.loginfo("Performance test started")
        self._hand_finder = HandFinder()
        self._hand_e = self._hand_finder.hand_e_available()
        self._hand_h = self._hand_finder.hand_h_available()
        rospy.loginfo("Hand E available: {}".format(self._hand_e))
        rospy.loginfo("Hand H available: {}".format(self._hand_h))

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._diagnostics_agg_callback)
        if self._hand_e:
            rospy.Subscriber("/rh/debug_etherCAT_data", EthercatDebug, self._ethercat_data_callback)

        self.total_control_loop_overruns_count = 0
        self.total_recent_control_loop_overruns_count = 0
        self.total_invalid_packets_total_count = 0
        self.total_invalid_packets_recent = 0
        self._invalid_hand_e_packets_total = 0

    def _diagnostics_agg_callback(self, diagnostics_full_msg):
        diagnostics_control_loop_msg = None
        diagnostics_ethercat_msg_list = list()

        for msg in diagnostics_full_msg.status:
            if msg.name == "/Realtime Control Loop/Realtime Control Loop":
                diagnostics_control_loop_msg = msg

            if msg.name.startswith("/EtherCat/EtherCAT Slaves/H0_"):
                diagnostics_ethercat_msg_list.append(msg)

        self._set_control_loop_values(diagnostics_control_loop_msg)
        if self._hand_h:
            self._set_ethercat_values(diagnostics_ethercat_msg_list)

    def _ethercat_data_callback(self, msg):
        msg_list = list()
        msg_list.append(msg)
        self._set_ethercat_values(msg_list)

    def _set_ethercat_values(self, diagnostics_ethercat_msg_list):
        for diagnostics_ethercat_msg in diagnostics_ethercat_msg_list:
            if self._hand_h:
                for value_list in diagnostics_ethercat_msg.values:
                    if value_list.key == "Invalid Packets Total":
                        invalid_packets_total_count = value_list.value
                    if value_list.key == "Invalid Packets Recent":
                        invalid_packets_recent = value_list.value
            elif self._hand_e:
                if diagnostics_ethercat_msg.motor_data_type.data == 0:
                    self._invalid_hand_e_packets_total += 1
        if self._hand_h:
            self.total_invalid_packets_total_count = float(invalid_packets_total_count)
            self.total_invalid_packets_recent += float(invalid_packets_recent)

    def _set_control_loop_values(self, diagnostics_control_loop_msg):
        for value_list in diagnostics_control_loop_msg.values:
            if value_list.key == "Control Loop Overruns":
                control_loop_overruns_count = value_list.value
            if value_list.key == "Recent Control Loop Overruns":
                recent_control_loop_overruns_count = value_list.value

        self.total_control_loop_overruns_count = float(control_loop_overruns_count)
        self.total_recent_control_loop_overruns_count += float(recent_control_loop_overruns_count)

    def _print_results(self, duration):
        avg_recent_control_loop_overruns_count = self.total_recent_control_loop_overruns_count / duration
        avg_total_invalid_packets_recent = self.total_invalid_packets_recent / duration

        rospy.loginfo("Control Loop Overruns: {}".format(self.total_control_loop_overruns_count))
        rospy.loginfo("Recent Control Loop Overruns: {}".format(avg_recent_control_loop_overruns_count))
        if self._hand_h:
            rospy.loginfo("Invalid Packets Total: {}".format(self.total_invalid_packets_total_count))
            rospy.loginfo("Invalid Packets Recent: {}".format(avg_total_invalid_packets_recent))
        else:
            rospy.loginfo("Invalid Packets Total: {}".format(self._invalid_hand_e_packets_total))

    def run(self, duration=20.0):

        if not self._hand_h and not self._hand_e:
            rospy.logerr("Not recognize hand!")
            return False
        start_time = rospy.get_time()
        time = 0.0
        while time < duration and not rospy.is_shutdown():
            time = rospy.get_time() - start_time
            rospy.sleep(0.0001)

        self._print_results(duration)


if __name__ == "__main__":
    rospy.init_node('performance_test', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-it', dest='duration', type=float, default=20)
    args, unknown = parser.parse_known_args()
    pt = PerformanceTest()
    pt.run(duration=args.duration)
