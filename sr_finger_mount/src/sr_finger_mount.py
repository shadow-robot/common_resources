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
from sr_robot_msgs.msg import ShadowPST, BiotacAll
from std_msgs.msg import Float64, Float64MultiArray
import threading
import time
from sr_hand.tactile_receiver import TactileReceiver

from dynamic_reconfigure.server import Server
from sr_finger_mount.cfg import SrFingerMountConfig


class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(DeviceHandler, self).__init__()
        self._device_name = device
        self._fingers = fingers
        self._mount = mount
        self._start_idx = [0] * len(self._fingers)
        self._m_phase = [0] * len(self._fingers)
        self._oldsignal = [0] * len(self._fingers)
        self._freq = [1] * len(self._fingers)
        self._amp = [1] * len(self._fingers)
        self._samplerate = sd.query_devices(self._device_name, 'output')['default_samplerate']
        self._amp_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/amplitude', Float64MultiArray, queue_size=50)
        self._freq_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/frequency', Float64, queue_size=50)

    def run(self):
        self.start_piezo(self._fingers)

    def callback(self, outdata, frames, time, status):
        if status:
            rospy.logwarn(status)

        for i, finger in enumerate(self._fingers):
            if self._mount._amplitudes[finger] > self._mount.CONST_AMP_MIN * 1.01:
                for frame in range(frames):
                    phase_inc = 2*np.pi*self._freq[i]/self._samplerate
                    outdata[frame, i] = self._amp[i] * np.sin(self._m_phase[i])
                    self._m_phase[i] += phase_inc
                    '''
                    if finger == 'th':
                        self._amp_publisher.publish(outdata[frame, i])
                    '''
                    if np.sign(outdata[frame, i]) != np.sign(self._oldsignal[i]):
                        self._freq[i] = self._mount._frequencies[finger]
                        self._amp[i] = self._mount._amplitudes[finger]
                    self._oldsignal[i] = outdata[frame, i]
            else:
                for frame in range(frames):
                    outdata[frame, i] = 0
            '''
            if finger == 'th':
                msg = Float64MultiArray()
                msg.data = outdata[:, i]
                self._amp_publisher.publish(msg)
            '''
            self._start_idx[i] += frames

    def start_piezo(self, fingers):
        with sd.OutputStream(device=self._device_name, channels=len(fingers), callback=self.callback,
                             samplerate=self._samplerate):
            while not rospy.is_shutdown():
                continue


class SrFingerMount():

    CONST_AMP_MAX = 0.6
    CONST_AMP_MIN = 0.2
    CONST_FREQ_MIN = 1
    CONST_FREQ_MAX = 80

    CONST_PST_MAX = 1  # 1600 for real PSTs
    CONST_PST_MIN = 0  # 350 for real PSTs

    PST_THRESHOLD = 395.0
    PST_SATURATION = 700.0

    CONST_BIOTAC_MAX = 1
    CONST_BIOTAC_MIN = 0

    CONST_FINGERS = ["ff", "mf", "rf", "lf", "th"]
    CONST_ACCEPTABLE_DEVICE_NAMES = ["Boreas DevKit", "BOS1901-KIT"]
    CONST_CHANNELS_PER_DEVICE = 2

    def __init__(self, fingers, hand_id):
        self._used_fingers = fingers
        self._used_devices = []
        self._hand_id = hand_id
        self._start_time = 0
        self.start_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [rospy.get_time()]))
        self._prev_values = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self._contact_time = 0.5

        self._used_tactiles = TactileReceiver(self._hand_id).get_tactile_type()

        if self._used_tactiles == "PST":
            rospy.Subscriber("/"+self._hand_id+"/tactile", ShadowPST, self._pst_tactile_cb)
        elif self._used_tactiles == "biotac":

            from haptx_tactile_mapping.biotac_sp_minus_mapping import BiotacMapping
            from haptx_msgs.msg import Movables, Movable, Tactor, BiotacAllFloat
            
            BiotacMapping(self._hand_id)
            rospy.Subscriber("haptx_movables", Movables, self._biotac_tactile_cb)

        self._amplitudes = dict()
        self._frequencies = dict()

        if not set(self._used_fingers).intersection(set(self.CONST_FINGERS)):
            rospy.logerr("Failed to start node! Used fingers are not one of these: {}".format(self.CONST_FINGERS))
            return

        if not self._check_devices():
            return
        self.init_all()

    def _reconfigure(self, config, level):
        rospy.logwarn(config)
        
        return config

    def _pst_tactile_cb(self, data):
        if len(data.pressure) == len(self.CONST_FINGERS):

            normalized_pressure = 5 * [None]
            for i, press in enumerate(data.pressure):            
                normalized_pressure[i] = (press - self.PST_THRESHOLD)/(self.PST_SATURATION - self.PST_THRESHOLD)
                normalized_pressure[i] = min(max(normalized_pressure[i], 0), 1)
                if i == 1 or i == 2 or i == 3:
                    normalized_pressure[i] = 0
            
            # rospy.logwarn("{:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(normalized_pressure[0],normalized_pressure[1],normalized_pressure[2],normalized_pressure[3],normalized_pressure[4]))
            mapped_pst_values = dict(zip(self.CONST_FINGERS, normalized_pressure))
            fading_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
            fading_amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
            fading_frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

            for i, f in enumerate(self._used_fingers):

                fading_time[f] = rospy.get_time() - self.start_time[f]

                if self._prev_values[f] == 0 and mapped_pst_values[f] > 0.01:
                    self.start_time[f] = rospy.get_time()
                    rospy.logwarn("zeroing startime for {}".format(f)) 

                if fading_time[f] < self._contact_time:        
                    fading_factor = (self._contact_time-fading_time[f])/self._contact_time           
                    fading_amplitudes[f] = self.CONST_AMP_MAX * 2.0 * fading_factor
                    fading_frequencies[f] = self.CONST_FREQ_MAX * fading_factor

                self._amplitudes[f] = ((mapped_pst_values[f] - self.CONST_PST_MIN) /
                                    (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                                    (self.CONST_AMP_MAX - self.CONST_AMP_MIN) + self.CONST_AMP_MIN
                self._frequencies[f] = ((mapped_pst_values[f] - self.CONST_PST_MIN) /
                                    (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                                    (self.CONST_FREQ_MAX - self.CONST_FREQ_MIN) + self.CONST_FREQ_MIN

                self._amplitudes[f] = max(self._amplitudes[f], fading_amplitudes[f])
                self._frequencies[f] = max(self._frequencies[f], fading_frequencies[f])
        
                self._prev_values[f] = mapped_pst_values[f]

        else:
            rospy.logwarn("Missing data. Expected to receive {}, but got {} PST values".format(len(self.CONST_FINGERS),
                                                                                               len(data.pressure)))

    def _biotac_tactile_cb(self, data):
        if len(data.movables) == len(self.CONST_FINGERS):
            pressure = 5 * [0.0]
            if self._hand_id in data.movables[0].name:
                for i in range(0, len(data.movables)):
                    pressure[i] = data.movables[i].tactors[0].pressure
                mapped_biotac_values = dict(zip(self.CONST_FINGERS, pressure))
                for f in self._used_fingers:
                    self._amplitudes[f] = ((mapped_biotac_values[f] - self.CONST_BIOTAC_MIN) /
                                           (self.CONST_BIOTAC_MAX - self.CONST_BIOTAC_MIN)) * \
                                           (self.CONST_AMP_MAX - self.CONST_AMP_MIN) + self.CONST_AMP_MIN
                    self._frequencies[f] = ((mapped_biotac_values[f] - self.CONST_BIOTAC_MIN) /
                                            (self.CONST_BIOTAC_MAX - self.CONST_BIOTAC_MIN)) * \
                        (self.CONST_FREQ_MAX - self.CONST_FREQ_MIN) + self.CONST_FREQ_MIN
        else:
            rospy.logwarn("Missing data. Expected to receive {}, "
                          "but got {} Biotac values".format(len(self.CONST_FINGERS), len(data.pressure)))

    def _check_devices(self):
        needed_devices = math.ceil(len(self._used_fingers)/2)
        device_list = sd.query_devices()
        present_devices = 0

        for acceptable_device in self.CONST_ACCEPTABLE_DEVICE_NAMES:
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
        for i in range(0, len(self._used_fingers), self.CONST_CHANNELS_PER_DEVICE):
            yield self._used_fingers[i:i + self.CONST_CHANNELS_PER_DEVICE]

    def init_all(self):
        device_handlers = [None, None]
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            time.sleep(1)
            device_handlers[i] = DeviceHandler(device_name, finger_set, self)
            device_handlers[i].start()


if __name__ == "__main__":

    rospy.init_node('sr_finger_mount')

    fingers = rospy.get_param("~fingers", 'th,ff,mf,rf,lf')
    hand_id = rospy.get_param("~hand_id", 'rh')

    if not (hand_id == "rh" or hand_id == "lh"):
        raise ValueError('/hand_id is not rh or lh')

    if fingers is not None:
        fingers = fingers.split(',')

    mount = SrFingerMount(fingers, hand_id)

    srv = Server(SrFingerMountConfig, mount._reconfigure)

    rospy.spin()
