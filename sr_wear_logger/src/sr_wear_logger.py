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

    '''
    def _updateAccumulatedValues(self, new_values):
        for key in self.currentValues.keys():
            self.currentValues[key] += new_values[key]
    '''
    def callback(self, msg):
        hand_data = self._extractHandData(msg)
        updates = dict.fromkeys(hand_data.keys(), 0)

        if self.first_run:
            self.previousValues = hand_data
            self.first_run = False

        timeAdded = False
        for key, value in hand_data.items():
            valueDifference = abs(self.previousValues[key] - value)
            if valueDifference > self.threshold:
                self.currentValues[key] += round(valueDifference, 5)
                if not timeAdded:
                    self.currentTime += 1.0/200 #frequency of rostopic being published
                    timeAdded = True 
        self.previousValues = hand_data

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
        print("Verify if there is data to save:",self._verifyData())
        if self._verifyData():
            f = open(self.logFilePath+self.logFileName, 'w')

            completeData = dict()
            completeData['total_angles'] = self.currentValues
            completeData['total_time'] = self.currentTime

            yaml.safe_dump(completeData, f)
            print("Saved to file!")
            f.close()

    def __init__(self):
        rospy.init_node('sr_wear_logger_node')
        self.first_run = True
        self.previousValues = {}
        self.currentValues = {}
        self.threshold = 0.0175

        self.logFilePath = "../data/"
        self.logFileName = "wear_data.yaml"
        
        self._initLog()
        rospy.Timer(rospy.Duration(5),self._saveDataLocaly)
        rospy.Timer(rospy.Duration(10),self._uploadToAWS)

        rospy.Subscriber('/joint_states', JointState, self.callback)

    def _verifyData(self):
        if len(self.currentValues.keys()) == 0:
            return False
        return True

    # this will be used to compare data between local file and AWS file
    # the file with higher values (newest) should be taken to init the variables
    def _updateLog(self, fileName_1, fileName_2):
        pass 

    def _loadDataFromYAML(self):
        f = open(self.logFilePath+self.logFileName, 'r')
        completeData = yaml.load(f, Loader=yaml.SafeLoader)            
        self.currentValues = completeData['total_angles']
        self.currentTime = completeData['total_time']
        f.close()

    def _initLog(self):
        if not os.path.exists(self.logFilePath):
            if not self._downloadFromAWS():
                pass
                #os.makedirs(self.logFilePath)
            
        if os.path.exists(self.logFilePath + self.logFileName):
            print("File exists locally - loading data")

            # shutil.copyfile(self.logFilePath + self.logFileName, self.logFilePath + "/wear_data_copy.yaml")
            # if not self._downloadFromAWS():
                #print("filed there - aws failed")     
            # self._updateLog(self.logFilePath + self.logFileName, self.logFilePath + "/wear_data_copy.yaml")
            
            self._loadDataFromYAML()

        else:
            print("Initiating new file!")
            f = open(self.logFilePath+self.logFileName, 'w')
            msg = rospy.wait_for_message('/joint_states', JointState)

            self.currentValues = dict.fromkeys(self._extractHandData(msg).keys(), 0)
            self.currentTime = 0
            
            completeData = dict()
            completeData['total_angles'] = self.currentValues
            completeData['total_time'] = self.currentTime
            yaml.safe_dump(completeData, f)     
            f.close()
            self._uploadToAWS()  
                 
        print("Initiated logfile!")



if __name__ == "__main__":
    logger = WearLogger()
    rospy.spin()
