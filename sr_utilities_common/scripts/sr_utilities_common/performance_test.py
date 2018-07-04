#!/usr/bin/python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_utilities.hand_finder import HandFinder
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug


class PerformanceTest(object):
    def __init__(self):
        rospy.loginfo("Performance test started")
        self.total_control_loop_overruns_count = 0
        self.total_recent_control_loop_overruns_count = 0
        self.total_invalid_packets_total_count = 0
        self.total_invalid_packets_recent = 0

    def set_ethercat_values(self, diagnostics_ethercat_msg_list):
        for diagnostics_ethercat_msg in diagnostics_ethercat_msg_list:
            for value_list in diagnostics_ethercat_msg.values:
                if value_list.key == "Invalid Packets Total":
                    invalid_packets_total_count = value_list.value
                if value_list.key == "Invalid Packets Recent":
                    invalid_packets_recent = value_list.value

        self.total_invalid_packets_total_count = float(invalid_packets_total_count)
        self.total_invalid_packets_recent += float(invalid_packets_recent)

    def set_control_loop_values(self, diagnostics_control_loop_msg):
        for value_list in diagnostics_control_loop_msg.values:
            if value_list.key == "Control Loop Overruns":
                control_loop_overruns_count = value_list.value
            if value_list.key == "Recent Control Loop Overruns":
                recent_control_loop_overruns_count = value_list.value

        self.total_control_loop_overruns_count = float(control_loop_overruns_count)
        self.total_recent_control_loop_overruns_count += float(recent_control_loop_overruns_count)

    def run(self):
        rospy.loginfo("Got into run...")
        hand_finder = HandFinder()
        rospy.loginfo("Hand E available: {}".format(hand_finder.hand_e_available()))
        rospy.loginfo("Hand H available: {}".format(hand_finder.hand_h_available()))
        # hand_finder.hand_e_available()
        for i in range(20):
            diagnostics_control_loop_msg = None
            diagnostics_ethercat_msg_list = list()

            diagnostics_full_msg = rospy.wait_for_message("/diagnostics_agg", DiagnosticArray)

            for msg in diagnostics_full_msg.status:
                # rospy.loginfo("name: {}".format(msg.name))
                if msg.name == "/Realtime Control Loop/Realtime Control Loop":
                    diagnostics_control_loop_msg = msg
                if hand_finder.hand_e_available():
                    ethercat_data = rospy.wait_for_message("/rh/debug_etherCAT_data", EthercatDebug)
                elif hand_finder.hand_h_available():
                    if msg.name.startswith("/EtherCat/EtherCAT Slaves/H0_"):
                        # rospy.loginfo("name: {}".format(msg.name))
                        diagnostics_ethercat_msg_list.append(msg)
                else:
                    rospy.logerr("Not recognized hand!")

            self.set_ethercat_values(diagnostics_ethercat_msg_list)
            self.set_control_loop_values(diagnostics_control_loop_msg)

        avg_recent_control_loop_overruns_count = self.total_recent_control_loop_overruns_count / 20.0
        avg_total_invalid_packets_recent = self.total_invalid_packets_recent / 20.0

        rospy.loginfo("Control Loop Overruns: {}".format(self.total_control_loop_overruns_count))
        rospy.loginfo("Recent Control Loop Overruns: {}".format(avg_recent_control_loop_overruns_count))
        rospy.loginfo("Invalid Packets Total: {}".format(self.total_invalid_packets_total_count))
        rospy.loginfo("Invalid Packets Recent: {}".format(avg_total_invalid_packets_recent))


if __name__ == "__main__":
    rospy.init_node('performance_test', anonymous=True)
    pt = PerformanceTest()
    pt.run()
