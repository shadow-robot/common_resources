#! /usr/bin/env python


import rospy
import os
import yaml
from sensor_msgs.msg import JointState

class WearLogger():

    def extractHandData(self, msg):
        hand_data = dict(zip(msg.name, msg.position))
        for key in hand_data.keys():
            if key.startswith("ra") or key.startswith("la"):
                hand_data.pop(key, None)
        return hand_data

    def updateAccumulatedValues(self, new_values):
        for key in self.currentValues.keys():
            self.currentValues[key] += new_values[key]
        
    def callback(self, msg):

        hand_data = self.extractHandData(msg)
        updates = dict.fromkeys(hand_data.keys(), 0)

        if self.first_run:
            self.previousValues = hand_data
            self.first_run = False

        for key, value in hand_data.items():
            valueDifference = abs(self.previousValues[key] - value)
            if valueDifference > self.threshold:
                updates[key] += round(valueDifference,5)
        self.previousValues = hand_data

        self.updateAccumulatedValues(updates)

    def __saveData(self):
        f = open(self.logFilePath,'w')
        yaml.safe_dump(self.currentValues, f)
        f.close()
        print(self.currentValues)
        
    def __init__(self):
        rospy.init_node('sr_wear_logger_node')
        rospy.on_shutdown(self.__saveData)

        self.first_run = True
        self.previousValues = {}
        self.currentValues = {}
        self.threshold = 0.001
        
        self.logFilePath = "../data/wear_data.yaml"
        self.initLog()

        rospy.Subscriber('/joint_states', JointState, self.callback)

    def initLog(self):
        if os.path.exists(self.logFilePath):
            print("File exists")
            f = open(self.logFilePath,'r')
            self.currentValues = yaml.load(f)      
        else:
            print("Creating file")
            f = open(self.logFilePath,'w')

            msg = rospy.wait_for_message('/joint_states', JointState, 2)
            self.currentValues = dict.fromkeys(self.extractHandData(msg).keys(), 0)
            yaml.safe_dump(self.currentValues, f)
            f.close()

if __name__ == "__main__":
    logger = WearLogger()
    rospy.spin()