import argparse
import sys
import math

import rospy
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from sr_robot_msgs.msg import ShadowPST
from std_msgs.msg import Float32
import threading
import time
import copy


# Every device has 2 channels 
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
        # for loop only for plotting 
        for f in self.fingers:
            self.mount._collection[f] = list()

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
                outdata[frame,i] = self.amp[i] * np.sin(self.m_phase[i]) #+ amp/2
                self.m_phase[i] += phaseInc

                if np.sign(outdata[frame,i]) != np.sign(self.oldsignal[i]):
                    self.freq[i] = self.mount._fm[finger]
                    self.amp[i] = self.mount._am[finger]
                self.oldsignal[i] = outdata[frame,i]

            # only for plotting
            self.mount._collection[finger].extend(outdata[:,i])

    def start_piezo(self, fingers):
        self.samplerate = sd.query_devices(self.device_name, 'output')['default_samplerate']
        with sd.OutputStream(device=self.device_name, channels=len(fingers), callback=self.callback,
                             samplerate=self.samplerate):
            #while not rospy.is_shutdown():
            #    continue

            input("Press anything to finish...")
            for i, f in enumerate(fingers):
                plt.subplot(len(fingers), 1, i+1)
                plt.plot(self.mount._collection[f])
            plt.show()

    
class SrFingerMount():
    def __init__(self, fingers, hand_id = "rh"):
        self._used_fingers = fingers
        self._fingers = ["th", "ff", "mf", "rf", "lf"]
        self._pst_max = 200 # 950
        self._pst_min = 1 # 350

        self._acceptable_device_names = ["Boreas DevKit", "BOS1901-KIT"]
        self._used_devices = []

        self.sub = rospy.Subscriber("/"+hand_id+"/tactile", ShadowPST, self._tactile_cb)

        self._amp_max = 1
        self._amp_min = 0.2
        self._freq_min = 1
        self._freq_max = 25

        self._am = dict()
        self._fm = dict()

        # ONLY FOR PLOTTING PURPOSES
        self._collection = dict(zip(fingers, list(len(fingers)*list())))

        if not set(self._used_fingers).intersection(set(self._fingers)):
            rospy.logwarn("Verify used fingers!")
            return 

        if self.check_devices():
            self.init_all()

    def _tactile_cb(self, data):
        if len(data.pressure) == 5:
            mapped_pst_values = dict(zip(self._fingers, data.pressure))

            for f in self._used_fingers:
                self._am[f] = ((mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
                               (self._amp_max - self._amp_min) + self._amp_min
                self._fm[f] = ((mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
                               (self._freq_max - self._freq_min) + self._freq_min
        else:
            rospy.logwarn("Missing values")

    # checks if there are enough devices to handle the given amount of fingers
    def check_devices(self):
        needed_devices = math.ceil(len(self._used_fingers)/2)
        device_list = sd.query_devices()
        present_devices = 0

        for acceptable_device in self._acceptable_device_names:
            for device in device_list:
                if acceptable_device in device['name']:
                    self._used_devices.append(device['name'])
                    present_devices += 1

        if needed_devices > present_devices:
            rospy.logerr("Not enough dev kits ({}/{}) connected to handle {} fingers".format(present_devices,needed_devices, len(self._used_fingers)))
            return False

        return True

    def _sublists(self):
        channels_per_device = 2
        for i in range(0, len(self._used_fingers), channels_per_device):
            yield self._used_fingers[i:i + channels_per_device]

    def init_all(self):
        x = [None, None]
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            rospy.logwarn(i)
            rospy.logwarn(device_name)
            rospy.logwarn(finger_set)
            x[i] = DeviceHandler(device_name, finger_set, self)
            time.sleep(1)
            x[i].start()

if __name__ == "__main__":

    def int_or_str(text):
        """Helper function for argument parsing."""
        try:
            return int(text)
        except ValueError:
            return text

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '-l', '--list-devices', action='store_true',
        help='show list of audio devices and exit')
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sd.query_devices())
        parser.exit(0)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[parser])
    parser.add_argument(
        'frequency', nargs='?', metavar='FREQUENCY', type=float, default=500,
        help='frequency in Hz (default: %(default)s)')
    parser.add_argument(
        '-d', '--device', type=int_or_str,
        help='output device (numeric ID or substring)')
    parser.add_argument(
        '-a', '--amplitude', type=float, default=0.2,
        help='amplitude (default: %(default)s)')
    args = parser.parse_args(remaining)

    rospy.init_node("sr_finger_mount_test")
    mount = SrFingerMount(["ff"])

