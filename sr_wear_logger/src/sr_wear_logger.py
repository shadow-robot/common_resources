#! /usr/bin/env python

import rospy
import os
import yaml
import roslaunch
import requests
import shutil
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
                updates[key] += round(valueDifference, 5)
        self.previousValues = hand_data
        self._updateAccumulatedValues(updates)

    def _uploadToAWS(self, event = None):
        launch_file_path = ('/home/user/projects/shadow_robot/base/src'
                            '/common_resources/sr_wear_logger/launch/sr_wear_logger_launch.launch')
        cli_args = [launch_file_path, 'upload:=true', 'output:=log']
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        rospy.sleep(2)
        parent.shutdown()       
        return rospy.get_param('aws_upload_succeeded')

    def _downloadFromAWS(self):
      
        launch_file_path = ('/home/user/projects/shadow_robot/base/src'
                            '/common_resources/sr_wear_logger/launch/sr_wear_logger_launch.launch')
        cli_args = [launch_file_path, 'download:=true']
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        rospy.sleep(2)
        parent.shutdown()
        return rospy.get_param('aws_download_succeeded')

    def _saveDataLocaly(self, event):
        print("Data to save:",self._verifyData())
        if self._verifyData():
            f = open(self.logFilePath+self.logFileName, 'w')
            yaml.safe_dump(self.currentValues, f)
            print("Saved to file!")
            f.close()

    def __init__(self):
        self.first_run = True
        self.previousValues = {}
        self.currentValues = {}
        self.threshold = 0.00175

        self.logFilePath = "../data/"
        self.logFileName = "wear_data.yaml"
        
        self._initLog()
        rospy.Timer(rospy.Duration(5),self._saveDataLocaly)
        rospy.Timer(rospy.Duration(10),self._uploadToAWS)

        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback)

    def _verifyData(self):
        if len(self.currentValues.keys()) == 0:
            return False
        return True

    def _updateLog(self, fileName_1, fileName_2):

        f1 = open(fileName_1, 'r')
        f2 = open(fileName_2, 'r')
        
        l1 = yaml.load(f1)
        l2 = yaml.load(f2)

        #print(l1)
        print("###################################")
        #print(l2)
        #for key in self.currentValues.keys():
            #self.currentValues[key] = max(l1[key], l2[key])
            #print(l1[key], l2[key], max(l1[key], l2[key]))
        print("###################################")

        f1.close()
        f2.close()


    def _initLog(self):

        print("path exists?:"+str(os.path.exists(self.logFilePath)))
        if not os.path.exists(self.logFilePath):
            print("path doesn't exist. attempting to download from aws")
            if not self._downloadFromAWS():
                print("no file - aws failed")
            
            
        if os.path.exists(self.logFilePath + self.logFileName):
            print("file exists - loading data")

            shutil.copyfile(self.logFilePath + self.logFileName, self.logFilePath + "/wear_data_copy.yaml")
            if not self._downloadFromAWS():
                print("filed there - aws failed")
                
            #self._updateLog(self.logFilePath + self.logFileName, self.logFilePath + "/wear_data_copy.yaml")

            #f = open(self.logFilePath+self.logFileName, 'r')
            #self.currentValues = yaml.load(f)
            #f.close()

        else:
            print("Initiating new file!")
            f = open(self.logFilePath+self.logFileName, 'w')
            msg = rospy.wait_for_message('/joint_states', JointState)
            print(msg)
            self.currentValues = dict.fromkeys(self._extractHandData(msg).keys(), 0)
            yaml.safe_dump(self.currentValues, f)     
            f.close()
            self._uploadToAWS()       

        print("Initiated logfile!")



if __name__ == "__main__":
    rospy.init_node('sr_wear_logger_node')
    logger = WearLogger()

    rospy.spin()

    print("sleeping")
    rospy.sleep(2)
    rospy.signal_shutdown("")
