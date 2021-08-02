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

m_phase1 = 0
m_phase2 = 0

#amp = np.arange(100)/100
#amp = np.random.randint(0, 100, 2000)/100

#print(amp, len(amp))
amphead = 0
amphead_inc = True
amphead_dec = False

#freq = np.arange(1000, 2500)
freq = np.random.randint(50, 150, 2000)
#freq = np.zeros(2000)+0

#print(freq, len(freq))
freqhead = 0
freqhead_inc = True
freqhead_dec = False

collection_1 = []
collection_2 = []

fc = 10
#amplitude = amp[0]
oldsignal = 0
start_idx = 0
thr_name = ""
# Every device has 2 channels 
# 
class DevHandler():
    def __init__(self, device, fingers):
        self.device_name = device
        self.fingers = fingers

    
class SrFingerMount():
    def __init__(self, fingers, hand_id = "rh"):
        self._used_fingers = fingers
        self._fingers = ["th", "ff", "mf", "rf", "lf"]
        self._pst_max = 950
        self._pst_min = 350
        self._mapped_pst_values = dict()

        self._acceptable_device_names = ["Boreas DevKit", "BOS1901-KIT"]
        self._used_devices = []

        self.sub = rospy.Subscriber("/"+hand_id+"/tactile", ShadowPST, self.tactile_cb)

        self._amp_max = 1
        self._amp_min = 10
        self._freq_min = 1
        self._freq_max = 10

        self._am = [None] * 5
        self._fm = [None] * 5

        if set(self._used_fingers).intersection(set(self._fingers)):
            pass
        else:
            rospy.logwarn("Verify used fingers!")
            exit(0)

        self.check_devices()
        self.init_all()

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
            exit(0)         
        else:
            rospy.logerr("Using {} devices [{}]".format(len(self._used_devices), self._used_devices))   

    def callback(self, outdata, frames, time, status):
        rospy.logwarn(thr_name)
        if status:
            print(status, file=sys.stderr)
        global start_idx
        t = (start_idx + np.arange(frames)) / 8000 # samplerate
        t = t.reshape(-1, 1)

        outdata[:] = args.amplitude/2 * np.sin(2 * np.pi * args.frequency * t) + args.amplitude/2

        rospy.logwarn("prinitn")
        rospy.logwarn(type(outdata))
        rospy.logwarn(outdata.shape)
        rospy.logwarn(type(t))
        rospy.logwarn(t.shape)
        #rospy.logwarn(outdata)

        start_idx += frames
    

    def _sublists(self):
        channels_per_device = 2
        for i in range(0, len(self._used_fingers), channels_per_device): 
            yield self._used_fingers[i:i + channels_per_device]

    def start_piezo(self, device_name, fingers):         
        thr_name = str(fingers)       
        with sd.OutputStream(device=device_name, channels=len(fingers), callback=self.callback,
                         samplerate=8000):
            print('#' * 80)
            print('press Return to quit')
            print('#' * 80)
            input()

    def init_all(self):
        x = [None, None]
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            rospy.logwarn(i)
            rospy.logwarn(device_name)
            rospy.logwarn(finger_set)

            samplerate = sd.query_devices(device_name, 'output')['default_samplerate']
            rospy.logerr(samplerate)
            x[i] = threading.Thread(target=self.start_piezo, args=(device_name,finger_set))
            x[i].start()

    def tactile_cb(self, data):        
        if len(data.pressure) == 5:
            self._mapped_pst_values = dict(zip(self._fingers, data.pressure))

            for idx, f in enumerate(list(self._used_fingers)):
                self._am[idx] = ((self._mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
                                (self._amp_max - self._amp_min) + self._amp_min
                self._fm[idx] = ((self._mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
                                (self._freq_max - self._freq_min) + self._freq_min
        else:
            rospy.logwarn("Missing values")
        

rospy.init_node("sr_finger_mount_test")
f_1 = SrFingerMount(["th", "ff"])


while not rospy.is_shutdown():
    rospy.spin()
    rospy.Rate(10).sleep()