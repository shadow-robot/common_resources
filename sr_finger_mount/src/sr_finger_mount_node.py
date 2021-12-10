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
from std_msgs.msg import Float64, Float64MultiArray, Header
import threading
import time
from sr_hand.tactile_receiver import TactileReceiver
import matplotlib.pyplot as plt

from dynamic_reconfigure.server import Server
from sr_finger_mount.cfg import SrFingerMountConfig

from sr_finger_mount.msg import PiezoFeedback


class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(DeviceHandler, self).__init__()
        self._device_name = device
        self._fingers = fingers
        self._mount = mount
        self._stop = False
        self._m_phase = [0] * len(self._fingers)
        self._oldsignal = [0] * len(self._fingers)
        self._freq = [1] * len(self._fingers)
        self._amp = [1] * len(self._fingers)
        self._samplerate = sd.query_devices(self._device_name, 'output')['default_samplerate']
        self._plot_buffer = list()
        self._clipped_amplitude = [-0.1, 0.6]
        self._offset = np.mean(self._clipped_amplitude)
        self._amp_factor = np.mean(np.abs(self._clipped_amplitude))

        self.signal_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/output', PiezoFeedback, queue_size=1)
        self.amp_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/amplitude', PiezoFeedback, queue_size=1)
        self.freq_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/frequency', PiezoFeedback, queue_size=1)

        self.msg = PiezoFeedback()
        self.msg.header = Header()

    def run(self):
        self.start_piezo(self._fingers)

    def callback(self, outdata, frames, time, status):
        if status:
            rospy.logwarn(status)

        for i, finger in enumerate(self._fingers):
            for frame in range(frames):
                phase_inc = 2 * np.pi * self._freq[i] / self._samplerate
                outdata[frame, i] = self._offset + self._amp_factor * self._amp[i] * np.sin(self._m_phase[i])
                self._m_phase[i] += phase_inc

                if outdata[frame, i] != np.sign(self._oldsignal[i]):
                    self._freq[i] = self._mount._frequencies[finger]
                    self._amp[i] = self._mount._amplitudes[finger]
                self._oldsignal[i] = outdata[frame, i]

                self.msg.header.stamp = rospy.get_rostime()
                self.msg.feedback = Float64(outdata[frame, i])
                self.signal_publisher.publish(self.msg)

    def start_piezo(self, fingers):
        with sd.OutputStream(device=self._device_name, channels=len(fingers), callback=self.callback,
                             samplerate=self._samplerate, blocksize=100, latency='low'):
            while not rospy.is_shutdown():
                rospy.sleep(0.5)


class SrFingerMount():

    CONST_PST_MAX = 1  # 1600 for real PSTs
    CONST_PST_MIN = 0  # 350 for real PSTs

    CONST_BIOTAC_MAX = 1
    CONST_BIOTAC_MIN = 0

    CONST_FINGERS = ["ff", "mf", "rf", "lf", "th"]
    CONST_ACCEPTABLE_DEVICE_NAMES = ["Boreas DevKit", "BOS1901-KIT"]
    CONST_CHANNELS_PER_DEVICE = 2

    def __init__(self, fingers, hand_id):
        self._used_fingers = fingers
        self._used_devices = []
        self._hand_id = hand_id

        self._contact_time = 0.1
        self._amp_max = 1.0
        self._amp_min = 0.0
        self._freq_min = 1
        self._freq_max = 80

        self._start_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [rospy.get_time()]))
        self.fading_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self._prev_values = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

        self._used_tactiles = TactileReceiver(self._hand_id).get_tactile_type()
        self._device_handlers = [None, None, None]

        if self._used_tactiles == "PST":
            # self._pst_threshold = [395.0, 395.0, 395.0, 395.0, 395.0]
            self._pst_saturation = [550.0, 550.0, 550.0, 550.0, 550.0]
            self._init_thresholds()

            rospy.Subscriber("/"+self._hand_id+"/tactile", ShadowPST, self._pst_tactile_cb)

        elif self._used_tactiles == "biotac":

            try:
                from haptx_tactile_mapping.biotac_sp_minus_mapping import BiotacMapping
            except Exception:
                rospy.logerr("haptx_tactile_mapping not found")

            try:
                from haptx_msgs.msg import Movables, Movable, Tactor, BiotacAllFloat
            except Exception:
                rospy.logerr("haptx_msgs not found")

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

    def _init_thresholds(self):
        samples_to_collect = 20
        thresholds_to_set = samples_to_collect * [None]
        for i in range(0, samples_to_collect):
            data = rospy.wait_for_message("/"+self._hand_id+"/tactile", ShadowPST)
            thresholds_to_set[i] = [data.pressure]
        self._pst_threshold = np.mean(thresholds_to_set, axis=1)[0]

    def _reconfigure(self, config, level):
        self._contact_time = config.contact_time
        self._amp_max = config.max_amplitude
        self._amp_min = config.min_amplitude
        self._freq_max = config.max_frequency
        self._freq_min = config.min_frequency
        self._pst_saturation = len(self.CONST_FINGERS) * [config.pst_saturation]
        rospy.logwarn("reconfigured")

        return config

    def _pst_tactile_cb(self, data):
        if len(data.pressure) == len(self.CONST_FINGERS):

            normalized_pressure = 5 * [None]
            for i, press in enumerate(data.pressure):
                normalized_pressure[i] = (press - self._pst_threshold[i]) / \
                                         (self._pst_saturation[i] - self._pst_threshold[i])
                normalized_pressure[i] = min(max(normalized_pressure[i], 0), 1)

            self.mapped_pst_values = dict(zip(self.CONST_FINGERS, normalized_pressure))

            for i, f in enumerate(self._used_fingers):

                self.fading_time[f] = rospy.get_time() - self._start_time[f]

                if self._prev_values[f] < 0.03 and np.abs(self.mapped_pst_values[f]-self._prev_values[f]) > 0.02:
                    self._start_time[f] = rospy.get_time()
                    rospy.logwarn("zeroing startime for {}".format(f))

                if self.fading_time[f] <= self._contact_time:
                    a = -4 / (self._contact_time * self._contact_time)
                    b = -a * self._contact_time
                    fading_factor = a * self.fading_time[f] * self.fading_time[f] + b * self.fading_time[f]
                    self.fading_amplitudes[f] = self._amp_max * 1.0 * fading_factor
                    self.fading_frequencies[f] = self._freq_max * fading_factor
                else:
                    self.fading_amplitudes[f] = 0
                    self.fading_frequencies[f] = 0

                self._amplitudes[f] = ((self.mapped_pst_values[f] - self.CONST_PST_MIN) /
                                       (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                    (self._amp_max - self._amp_min) + self._amp_min
                self._frequencies[f] = ((self.mapped_pst_values[f] - self.CONST_PST_MIN) /
                                        (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                    (self._freq_max - self._freq_min) + self._freq_min

                self._amplitudes[f] = max(self._amplitudes[f], self.fading_amplitudes[f])
                self._frequencies[f] = max(self._frequencies[f], self.fading_frequencies[f])

                self._prev_values[f] = self.mapped_pst_values[f]

        else:
            rospy.logwarn("Missing data. Expected to receive {}, but got {} PST values".format(len(self.CONST_FINGERS),
                                                                                               len(data.pressure)))

    def _biotac_tactile_cb(self, data):
        if len(data.movables) == len(self.CONST_FINGERS):
            pressure = 5 * [0.0]
            if self._hand_id in data.movables[0].name:
                for i in range(0, len(data.movables)):
                    pressure[i] = data.movables[i].tactors[0].pressure

                self.mapped_biotac_values = dict(zip(self.CONST_FINGERS, pressure))

                for i, f in enumerate(self._used_fingers):

                    self.fading_time[f] = rospy.get_time() - self._start_time[f]

                    if self._prev_values[f] < 0.01 and np.abs(self.mapped_biotac_values[f]-self._prev_values[f]) > 0.01:
                        self._start_time[f] = rospy.get_time()
                        rospy.logerr("zeroing startime for {}".format(f))

                    if self.fading_time[f] <= self._contact_time:
                        a = -4 / (self._contact_time * self._contact_time)
                        b = -a * self._contact_time
                        fading_factor = a * self.fading_time[f] * self.fading_time[f] + b * self.fading_time[f]
                        self.fading_amplitudes[f] = self._amp_max * 1.0 * fading_factor
                        self.fading_frequencies[f] = self._freq_max * fading_factor
                    else:
                        self.fading_amplitudes[f] = 0
                        self.fading_frequencies[f] = 0

                    self._amplitudes[f] = ((self.mapped_biotac_values[f] - self.CONST_BIOTAC_MIN) /
                                           (self.CONST_BIOTAC_MAX - self.CONST_BIOTAC_MIN)) * \
                        (self._amp_max - self._amp_min) + self._amp_min
                    self._frequencies[f] = ((self.mapped_biotac_values[f] - self.CONST_BIOTAC_MIN) /
                                            (self.CONST_BIOTAC_MAX - self.CONST_BIOTAC_MIN)) * \
                        (self._freq_max - self._freq_min) + self._freq_min
                    '''
                    if self._amplitudes[f] != 0.0:
                        self._amplitudes[f] = self._amp_max
                        self._frequencies[f] = self._freq_max
                    else:
                        self._amplitudes[f] = 0
                        self._frequencies[f] = 0
                    '''
                    self._amplitudes[f] = max(self._amplitudes[f], self.fading_amplitudes[f])
                    self._frequencies[f] = max(self._frequencies[f], self.fading_frequencies[f])
                    self._prev_values[f] = self.mapped_biotac_values[f]
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
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            time.sleep(1)
            self._device_handlers[i] = DeviceHandler(device_name, finger_set, self)
            self._device_handlers[i].start()


if __name__ == "__main__":

    rospy.init_node('sr_finger_mount_node')

    fingers = rospy.get_param("~fingers", 'th,ff,mf,rf')
    hand_id = rospy.get_param("~hand_id", 'rh')

    if not (hand_id == "rh" or hand_id == "lh"):
        raise ValueError('/hand_id is not rh or lh')

    if fingers is not None:
        fingers = fingers.split(',')

    mount = SrFingerMount(fingers, hand_id)
    # srv = Server(SrFingerMountConfig, mount._reconfigure)
    rospy.spin()
