#! /usr/bin/env python


import rospy
import os
import yaml
import roslaunch
from sensor_msgs.msg import JointState

class WearLogger():

    def _extractHandData(self, msg):
        hand_data = dict(zip(msg.name, msg.position))
        for key in hand_data.keys():
            if key.startswith("ra") or key.startswith("la"):
                hand_data.pop(key, None)
        return hand_data

    def _updateAccumulatedValues(self, new_values):
        for key in self.currentValues.keys():
            self.currentValues[key] += new_values[key]
        
    def callback(self, msg):

        hand_data = self._extractHandData(msg)
        updates = dict.fromkeys(hand_data.keys(), 0)

        if self.first_run:
            self.previousValues = hand_data
            self.first_run = False

        for key, value in hand_data.items():
            valueDifference = abs(self.previousValues[key] - value)
            if valueDifference > self.threshold:
                updates[key] += round(valueDifference,5)

        self.previousValues = hand_data
        self._updateAccumulatedValues(updates)

    def _sendToAWS(self):
        launch_file_path = '/home/user/projects/shadow_robot/base/src/common_resources/sr_wear_logger/launch/sr_wear_logger_launch.launch'
        cli_args = [launch_file_path]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        rospy.sleep(10)
        parent.shutdown()

    def _saveData(self):
        print("eeee")
        f = open(self.logFilePath+self.logFileName,'w')
        yaml.safe_dump(self.currentValues, f)
        f.close()
        print(self.currentValues)
        #self._sendToAWS()
        
    def __init__(self):
        rospy.init_node('sr_wear_logger_node')
        rospy.on_shutdown(self._saveData)

        self.first_run = True
        self.previousValues = {}
        self.currentValues = {}
        self.threshold = 0.0175

        self.logFilePath = "../data/"
        self.logFileName = "wear_data.yaml"
        self._initLog()

        rospy.Subscriber('/joint_states', JointState, self.callback)

    def _initLog(self):
        if not os.path.exists(self.logFilePath):
            os.makedirs(self.logFilePath)
        
        if os.path.exists(self.logFilePath + self.logFileName):
            f = open(self.logFilePath+self.logFileName,'r')
            self.currentValues = yaml.load(f)                  
        else:
            f = open(self.logFilePath+self.logFileName,'w')
            msg = rospy.wait_for_message('/joint_states', JointState)
            self.currentValues = dict.fromkeys(self._extractHandData(msg).keys(), 0)
            yaml.safe_dump(self.currentValues, f)

        f.close()

if __name__ == "__main__":
    logger = WearLogger()
    rospy.spin()