#!/usr/bin/env python

# example use: ./overrun_experiments.py -ht hand_e

import rospy
import argparse
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug
import csv


class OverrunExperiment(object):
    def __init__(self, hand_type):
        self.hand_type = hand_type
        self.supported_hand_types = ['hand_e', 'hand_h']
        self.num_of_drops = 0
        self.iterations = 0

    def overruns_callback_hand_h(self, data):
        overrun = data.status[9].values[9].value
        rospy.loginfo(overrun)
        self.num_of_drops = sum(int(data.status[idx].values[8].value) for idx in range(4, 7))

        with open("overruns_data.txt", "a") as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.num_of_drops = 0
        self.iterations += 1

    def overruns_callback_hand_e(self, data):
        overrun = data.status[12].values[9].value
        with open("overruns_data.txt", "a") as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.num_of_drops = 0
        self.iterations += 1

    def drops_callback_hand_e(self, data):
        if (0 == data.sensors[10]):
            self.num_of_drops += 1

    def listener(self):
        if not self.hand_type:
            raise ValueError('Please specify hand type using -ht (--hand_type) flag!')
        elif self.hand_type not in self.supported_hand_types:
            raise ValueError('Unrecognized hand type: {}!'.format(self.hand_type))

        rospy.loginfo("Your data is being recorded, please wait...")
        if 'hand_h' == self.hand_type:
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_h)
        elif 'hand_e' == self.hand_type:
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_e)
            rospy.Subscriber("/rh/debug_etherCAT_data", EthercatDebug, self.drops_callback_hand_e)
        while (self.iterations < 30) and (not rospy.is_shutdown()):
            rospy.sleep(0.1)
        rospy.loginfo("Your data has been recorded to ./overruns_data.txt file.")


if __name__ == '__main__':
    rospy.init_node('overruns_experiment', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-ht', '--hand_type', type=str, help='Hand type, e.g. hand_e or hand_h')
    args = parser.parse_args()
    oe = OverrunExperiment(args.hand_type)
    oe.listener()
