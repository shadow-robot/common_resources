#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import requests
import actionlib
import os
import subprocess
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from std_msgs.msg import Bool
import sr_utilities_common.msg
from sr_utilities_common.msg import PowerManagerAction
from sr_utilities_common.msg import PowerManagerGoal


class Device:
  name = ""
  power_ip = ""
  device_ip = ""

class Struct:
    def __init__(self, **entries):
        self.__dict__.update(entries)


class RemotePowerControl(object):
    _feedback = sr_utilities_common.msg.PowerManagerFeedback()
    _result = sr_utilities_common.msg.PowerManagerResult()

    def __init__(self, name, arm_power_ip, arm_ip_address, devices, side="right"):
        self._action_name = name
        self._on_off_delay = 0.4
        self._arm_power_ip = arm_power_ip
        self._http_arm_power_ip = 'http://' + arm_power_ip
        self._arm_data_ip = arm_ip_address
        self._devices = devices
        #self._power_on_list = []
        #self._power_off_list = []
        rospy.Subscriber(side + "_arm_power_on", Bool, self.power_on_cb)
        rospy.Subscriber(side + "_arm_power_off", Bool, self.power_off_cb)
        self.goal = PowerManagerGoal()

        self._as = actionlib.SimpleActionServer(self._action_name, PowerManagerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        if self.does_ip_relay_respond():
            rospy.loginfo("Contacted " + side + " arm ip relay")
        else:
            rospy.logerr("Cannot reach arm ip relay at " + self._arm_power_ip)
        #while not rospy.is_shutdown():
            #rospy.spin()

    def execute_cb(self, goal):
        rospy.logwarn("cb")
        self.goal = goal
        # helper variables
        r = rospy.Rate(1)
        success = True
        self._power_on_list = goal.power_on
        self._power_off_list = goal.power_off

        for power_on_goal in goal.power_on:
            for device in self._devices:
                if power_on_goal == device.name:
                    if self.is_arm_off():
                        rospy.loginfo("powering on...")
                        self.power_on()

        for power_off_goal in goal.power_off:
            for device in self._devices:
                if power_off_goal == device.name:
                    if self.is_arm_on():
                        rospy.loginfo("powering off...")
                        self.power_off()
        
        # append the seeds for the fibonacci sequence
        for i in range(0, 2):
            fb = sr_utilities_common.msg.sr_power_feedback()
            fb.status = "test" + str(i)
            self._feedback.feedback.append(fb)
        
        # publish info to the console for the user
        rospy.logwarn('%s: Executing, creating fibonacci sequence of string %s with %s, %s' % (self._action_name, goal.power_on[0], self._feedback.feedback[0], self._feedback.feedback[1]))
        
        # start executing the action
        for i in range(1, 2):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.feedback.append(self._feedback.feedback[i].status + self._feedback.feedback[i-1].status)
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result.results.append(self._feedback.feedback[0])
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def requests_retry_session(self, retries=3, backoff_factor=0.3, status_forcelist=(500, 502, 504), session=None):
        session = session or requests.Session()
        retry = Retry(
            total=retries,
            read=retries,
            connect=retries,
            backoff_factor=backoff_factor,
            status_forcelist=status_forcelist,
        )
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)
        session.mount('https://', adapter)
        return session

    def power_on(self):
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[45]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[45]=0')
        rospy.sleep(self._on_off_delay)

    def power_off(self):
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[46]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[46]=0')
        rospy.sleep(self._on_off_delay)

    def power_on_cb(self, data):
        if data.data == True:
            if self.is_arm_off():
                rospy.loginfo("powering on...")
                self.power_on()
            else: 
                rospy.logwarn("arm already on, not powering up")

    def power_off_cb(self, data):
        if data.data == True:
            if self.is_arm_on():
                rospy.loginfo("powering off...")
                self.power_off()
            else:
                rospy.logwarn("Arm already off, not sending shutdown signal")

    def is_ping_successful(self, ip):
        command = 'fping -c1 -t500 ' + ip + ' 2>&1 >/dev/null'
        p = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        stdout = p.communicate()[0]
        if (p.returncode == 0):
            return True
        else:
            return False

    def does_ip_relay_respond(self):
        return self.is_ping_successful(self._arm_power_ip)

    def is_arm_off(self):
        return not self.is_ping_successful(self._arm_data_ip)

    def is_arm_on(self):
        return self.is_ping_successful(self._arm_data_ip)

    def is_arm_booting(self):        
        while True:
            t = os.system('fping -c1 -t500 192.168.1.1')
            print t





if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=False)
    config_path = os.path.join(rospkg.RosPack().get_path('sr_utilities_common'), 'config')    
    with open(config_path + 'power_devices.yaml') as f:
        # use safe_load instead load
        dataMap = yaml.safe_load(f)

    if rospy.has_param('~arm_ip'):
        arm_ip = rospy.get_param("~arm_ip")
    else:
        arm_ip = "192.168.1.1"

    if rospy.has_param('~arm_power_ip'):
        arm_power_ip = rospy.get_param("~arm_power_ip")
    else:
        arm_power_ip = "10.6.10.105"

    if rospy.has_param('~side'):
        side = rospy.get_param("~side")
    else:
        side = "right"
    ##ip_power_address = "http://" + arm_power_ip
    devices = []
    device = Device()
    device.name = "right_arm"
    device.power_ip = "10.6.10.105"
    device.device_ip = "192.168.1.1"
    devices.append(device)
    arm_address = arm_ip
    remote_power_control = RemotePowerControl(rospy.get_name(), arm_power_ip, arm_address, devices)

