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
import yaml
import rospkg
import subprocess
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from std_msgs.msg import Bool
import sr_utilities_common.msg
from sr_utilities_common.msg import PowerManagerAction
from sr_utilities_common.msg import PowerManagerGoal
from sr_utilities_common.srv import CustomRelayCommand,CustomRelayCommandResponse


class Device:
  name = ""
  power_ip = ""
  device_ip = ""

class Devices:
    def __init__(self, **entries):
        self.__dict__.update(entries)

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")


class RemotePowerControl(object):
    _feedback = sr_utilities_common.msg.PowerManagerFeedback()
    _result = sr_utilities_common.msg.PowerManagerResult()

    def __init__(self, name, devices):
        self._action_name = name
        self._on_off_delay = 0.4
        self._devices = devices
        self.goal = PowerManagerGoal()
        self._as = actionlib.SimpleActionServer(self._action_name, PowerManagerAction, execute_cb=self.execute_cb, auto_start = False)
        self._custom_relay_service = rospy.Service('custom_power_relay_command', CustomRelayCommand, self.handle_custom_relay_request)
        self._as.start()
        self.check_relays_connected()
        while not rospy.is_shutdown():
            rospy.spin()

    def handle_custom_relay_request(self, req):
        if 'get' in req.request_type.lower():
            req_type = 'get'
        if 'set' in req.request_type.lower():
            req_type = 'set'
        request_string = 'http://' + str(req.ip_address) + '/' + str(req_type) + 'para[' + str(req.param_number) + ']=' + str(req.value)
        response = self.requests_retry_session().get(request_string)
        response_content = response.content.replace("<html><body>", '').replace("</body></html>\r\n\r\n", '')
        return CustomRelayCommandResponse(response_content)

    def check_relays_connected(self):
        for device in self._devices:
            if 'arm' in device['name']:
                if self.does_ip_relay_respond(device['power_ip']):
                    rospy.loginfo("Contacted " + device['name'] + " ip relay at " + device['power_ip'])
                else:
                    rospy.logwarn("Could not contact " + device['name'] + " ip relay at " + device['power_ip'])

    def execute_cb(self, goal):
        rospy.logwarn("cb")
        self.goal = goal
        # helper variables
        r = rospy.Rate(1)
        success = True

        for power_on_goal in goal.power_on:
            for device in self._devices:
                if device['name'] == power_on_goal:
                    if not 'arm' in device['name'] or self.is_arm_off(device['data_ip']):
                        rospy.loginfo("powering on...")
                        self.power_on(device['power_ip'])

        for power_off_goal in goal.power_off:
            for device in self._devices:
                if device['name'] == power_off_goal:
                    if not 'arm' in device['name'] or self.is_arm_on(device['data_ip']):
                        rospy.loginfo("powering off...")
                        self.power_off(device['power_ip'])
        
        # append the seeds for the fibonacci sequence
        for i in range(0, 2):
            fb = sr_utilities_common.msg.sr_power_feedback()
            fb.status = "test" + str(i)
            self._feedback.feedback.append(fb)
        
        # publish info to the console for the user
        rospy.logwarn('%s: Executing. \nPower on list: %s \nPower off list: %s' % (self._action_name, goal.power_on, goal.power_off))
        
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

    def power_on(self, power_ip):
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[45]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[45]=0')
        rospy.sleep(self._on_off_delay)

    def power_off(self, power_ip):
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[46]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[46]=0')
        rospy.sleep(self._on_off_delay)

    def is_ping_successful(self, ip):
        command = 'fping -c1 -t500 ' + ip + ' 2>&1 >/dev/null'
        p = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        stdout = p.communicate()[0]
        if (p.returncode == 0):
            return True
        else:
            return False

    def does_ip_relay_respond(self, ip):
        return self.is_ping_successful(ip)

    def is_arm_off(self, ip):
        return not self.is_ping_successful(ip)

    def is_arm_on(self, ip):
        return self.is_ping_successful(ip)

    def is_arm_booting(self):        
        while True:
            t = os.system('fping -c1 -t500 192.168.1.1')
            print t


if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=False)
    config_file = 'power_devices.yaml'
    config_path = os.path.join(rospkg.RosPack().get_path('sr_utilities_common'), 'config')
    config = os.path.join(config_path, config_file)
    with open(config) as f:
        # use safe_load instead load
        dataMap = yaml.safe_load(f)

    remote_power_control = RemotePowerControl(rospy.get_name(), dataMap)


