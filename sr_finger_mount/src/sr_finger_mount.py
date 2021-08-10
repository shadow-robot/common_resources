#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
from __future__ import division
import argparse
import sys
import math

import rospy
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from sr_robot_msgs.msg import ShadowPST
import threading
import time


class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(DeviceHandler, self).__init__()
        self.device_name = device
        self.fingers = fingers
        self.mount = mount
        self.start_idx = [0] * len(self.fingers)
        self.m_phase = [0] * len(self.fingers)
        self.oldsignal = [0] * len(self.fingers)
        self.freq = [1] * len(self.fingers)
        self.amp = [1] * len(self.fingers)

    def run(self):
        '''
        # FOR-LOOP ONLY FOR PLOTTING PURPOSES
        for f in self.fingers:
            self.mount._collection[f] = list()
        '''
        self.start_piezo(self.fingers)

    def callback(self, outdata, frames, time, status):
        if status:
            print(status, file=sys.stderr)

        for i, finger in enumerate(self.fingers):

            t = (self.start_idx[i] + np.arange(frames)) / self.samplerate
            t = t.reshape(-1, 1)
            self.start_idx[i] += frames

            for frame in range(frames):
                phaseInc = 2*np.pi*self.freq[i]/self.samplerate
                outdata[frame, i] = self.amp[i] * np.sin(self.m_phase[i])
                self.m_phase[i] += phaseInc

                if np.sign(outdata[frame, i]) != np.sign(self.oldsignal[i]):
                    self.freq[i] = self.mount._fm[finger]
                    self.amp[i] = self.mount._am[finger]
                self.oldsignal[i] = outdata[frame, i]
            '''
            # ONLY FOR PLOTTING PURPOSES
            self.mount._collection[finger].extend(outdata[:, i])
            '''

    def start_piezo(self, fingers):
        self.samplerate = sd.query_devices(self.device_name, 'output')['default_samplerate']
        with sd.OutputStream(device=self.device_name, channels=len(fingers), callback=self.callback,
                             samplerate=self.samplerate):
            while not rospy.is_shutdown():
                continue
            '''
            # ONLY FOR PLOTTING PURPOSES
            input("Press anything to finish...")
            for i, f in enumerate(fingers):
                plt.subplot(len(fingers), 1, i+1)
                plt.plot(self.mount._collection[f])
            plt.show()
            '''


class SrFingerMount():
    def __init__(self, fingers, hand_id="rh"):
        self._used_fingers = fingers
        self._fingers = ["ff", "mf", "rf", "lf", "th"]
        self._pst_max = 200  # 950
        self._pst_min = 1  # 350

        self._acceptable_device_names = ["Boreas DevKit", "BOS1901-KIT"]
        self._used_devices = []

        self.sub = rospy.Subscriber("/"+hand_id+"/tactile", ShadowPST, self._tactile_cb)

        self._amp_max = 1.0
        self._amp_min = 0.2
        self._freq_min = 5
        self._freq_max = 30

        self._am = dict()
        self._fm = dict()
        '''
        # ONLY FOR PLOTTING PURPOSES
        self._collection = dict(zip(fingers, list(len(fingers)*list())))
        '''
        
        if not set(self._used_fingers).intersection(set(self._fingers)):
            rospy.logwarn("Verify used fingers!")
            return

        if self.check_devices():
            self.init_all()
            rospy.logerr("started")

    def _tactile_cb(self, data):
        if len(data.pressure) == 5:
            mapped_pst_values = dict(zip(self._fingers, data.pressure))

            for f in self._used_fingers:
                self._am[f] = ((mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min)) * \
                              (self._amp_max - self._amp_min) + self._amp_min
                self._fm[f] = ((mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min)) * \
                              (self._freq_max - self._freq_min) + self._freq_min
        else:
            rospy.logwarn("Missing values")

    def check_devices(self):
        needed_devices = math.ceil(len(self._used_fingers)//2)
        device_list = sd.query_devices()
        present_devices = 0

        for acceptable_device in self._acceptable_device_names:
            for device in device_list:
                if acceptable_device in device['name']:
                    self._used_devices.append(device['name'])
                    present_devices += 1

        if needed_devices > present_devices:
            rospy.logerr("Not enough dev kits ({}/{}) connected to handle {} fingers".format(present_devices,
                                                                                             needed_devices,
                                                                                             len(self._used_fingers)))
            return False
        return True

    def _sublists(self):
        channels_per_device = 2
        for i in range(0, len(self._used_fingers), channels_per_device):
            yield self._used_fingers[i:i + channels_per_device]

    def init_all(self):
        dh = [None, None]
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            dh[i] = DeviceHandler(device_name, finger_set, self)
            time.sleep(1)
            dh[i].start()


if __name__ == "__main__":

    fingers = rospy.get_param("~fingers", None)
    side = rospy.get_param("~side", None)

    if side == "rh" or side == "lh":
        rospy.init_node("sr_"+side+"_finger_mount")

    if fingers is not None:
        fingers = fingers.split(',')

    mount = SrFingerMount(fingers, side)
