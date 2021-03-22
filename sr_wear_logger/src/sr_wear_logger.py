#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class WearLogger():

    def callback(self, msg):
        #print(msg.position)
        
        for i in range(len(msg.position)):
            updatedValues = abs(self.counter[i] - msg.position[i])
            if updatedValues > self.threshold:
                self.accumulated[i] += round(updatedValues,4)

        counters = dict(zip(msg.name, self.accumulated))
        self.counter = msg.position     
        #print(msg.name)
        #print(type(self.accumulated[0]))
        print(counters['ra_wrist_3_joint'])

    def saveData(self):
        f = open("demofile3.txt", "w")
        f.write("Woops! I have deleted the content!")
        f.close()
        
    def __init__(self):
        rospy.init_node('sr_wear_logger_node')
        rospy.on_shutdown(self.saveData)
        msg = rospy.wait_for_message('/joint_states', JointState, 2)

        self.counters = {}
        self.accumulated = 40 * [0]
        self.counter = 40 * [0]
        self.threshold = 0.001

        rospy.Subscriber('/joint_states', JointState, self.callback)




if __name__ == "__main__":
    logger = WearLogger()
    rospy.spin()