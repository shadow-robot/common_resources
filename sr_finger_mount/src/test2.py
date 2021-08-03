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

m_phase = 0

#amp = np.arange(100)/100
#amp = np.random.randint(0, 100, 2000)/100

#print(amp, len(amp))
amphead = 0
amphead_inc = True
amphead_dec = False

freq = 1
amp = 1
#freq = np.random.randint(50, 150, 2000)
#freq = np.zeros(2000)+0

#print(freq, len(freq))
freqhead = 0
freqhead_inc = True
freqhead_dec = False

collection = []


#amplitude = amp[0]
oldsignal = 0
start_idx = 0


# Every device has 2 channels 

class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(DeviceHandler, self).__init__()
        self.device_name = device
        self.fingers = fingers
        self.mount = mount
    
    def run(self):
        self.start_piezo(self.device_name, self.fingers)

    def callback(self, outdata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        global start_idx
        t = (start_idx + np.arange(frames)) / self.samplerate 
        t = t.reshape(-1, 1)
        
        global m_phase, oldsignal, collection, freq, amp

        for idx, f in enumerate(self.fingers):      
            
            for i in range(frames):
                phaseInc = 2*np.pi*freq/self.samplerate
                outdata[i,idx] = amp * np.sin(m_phase) #+ amp/2
                m_phase += phaseInc
                #rospy.logwarn("Current frequency {} and {}".format(freq, phaseInc))

                if np.sign(outdata[i,idx]) != np.sign(oldsignal):
                    freq = self.mount._fm[f]
                    amp = self.mount._am[f]
                    #rospy.logwarn("Updating")
                    #rospy.logwarn(freq)
                    #rospy.logwarn(amp)
                    #collection += list([0.2])
                    
                oldsignal = outdata[i,idx]

            #collection += list(outdata[:,0])
            #collection += list([1])
        
        '''
        for idx, f in enumerate(self.fingers):
            outdata[:,idx] = list(self.mount._am[f]/2 * np.sin(2*np.pi*self.mount._fm[f]*t) + self.mount._am[f]/2)
        collection += list(outdata[:,0])
        '''
        start_idx += frames

    def start_piezo(self, device_name, fingers):       
        self.samplerate = sd.query_devices(device_name, 'output')['default_samplerate']  
        with sd.OutputStream(device=device_name, channels=len(fingers), callback=self.callback,
                         samplerate=self.samplerate):
            #while not rospy.is_shutdown():
            #    continue

            global collection 
            input()
            plt.subplot(1,1,1)
            plt.plot(collection)
            plt.show()

    
class SrFingerMount():
    def __init__(self, fingers, hand_id = "rh"):
        self._used_fingers = fingers
        self._fingers = ["th", "ff", "mf", "rf", "lf"]
        self._pst_max = 200 #950
        self._pst_min = 1 #350
        self._mapped_pst_values = dict()

        self._acceptable_device_names = ["Boreas DevKit", "BOS1901-KIT"]
        self._used_devices = []

        self.sub = rospy.Subscriber("/"+hand_id+"/tactile", ShadowPST, self.tactile_cb)

        self._amp_max = 1
        self._amp_min = 0.2
        self._freq_min = 1
        self._freq_max = 25

        self._am = dict()
        self._fm = dict()

        if set(self._used_fingers).intersection(set(self._fingers)):
            pass
        else:
            rospy.logwarn("Verify used fingers!")
            exit(0)

        self.check_devices()
        self.init_all()

    def tactile_cb(self, data):        
        if len(data.pressure) == 5:
            self._mapped_pst_values = dict(zip(self._fingers, data.pressure))

            for f in self._used_fingers:
                self._am[f] = ((self._mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
                                (self._amp_max - self._amp_min) + self._amp_min
                self._fm[f] = ((self._mapped_pst_values[f] - self._pst_min) / (self._pst_max - self._pst_min))* \
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
            exit(0)         
        else:
            rospy.logerr("Using {} devices [{}]".format(len(self._used_devices), self._used_devices))   

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


rospy.init_node("sr_finger_mount_test")
f_1 = SrFingerMount(["th"]) #, "ff", "mf"])


while not rospy.is_shutdown():
    rospy.spin()
    rospy.Rate(10).sleep()