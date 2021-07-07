#!/usr/bin/env python3
"""Play a sine signal."""
import argparse
import sys
import rospy
import matplotlib.pyplot as plt

import numpy as np
import sounddevice as sd
from sr_robot_msgs.msg import ShadowPST
from std_msgs.msg import Float32

global thumb_press, amp, freq, start_idx
global modulating_freq, modulating_amp, modulating_signal
global j, total_1, total_2, total_3
j = 0
total_1 = []
total_2 = []
total_3 = []

carrier_freq = 20
carrier_amp = 1
carrier_signal = []

modulating_freq = 50
modulating_amp = 0.2
modulating_signal = []


class SrFingerMount():
    def tactile_cb(self, data):        
        global thumb_press, amp, freq
        thumb_press = data.pressure[0]
        #thumb_max = 950
        #thumb_min = 330
        thumb_max = 200
        thumb_min = 0
        amp_max = 1.0
        amp_min = 0.2
        freq_min = 5
        freq_max = 25
        amp = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (amp_max - amp_min) + amp_min
        freq = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (freq_max - freq_min) + freq_min


    def __init__(self):

        rospy.init_node("sr_finger_mount_test")
        samplerate = sd.query_devices(args.device, 'output')['default_samplerate']
        rospy.Subscriber("/rh/tactile", ShadowPST, self.tactile_cb)
        rospy.wait_for_message("/rh/tactile", ShadowPST)

        def callback(outdata, frames, time, status):
            if status:
                print(status, file=sys.stderr)

            global total_1, total_2, total_3 , start_idx

            t = (start_idx+np.arange(frames)) / samplerate
            t = t.reshape(-1, 1)

            modulating_amp = 1
            modulating_freq = freq

            carrier_signal = carrier_amp*np.sin(t*carrier_freq)
            modulating_signal = modulating_amp*np.sin(t*modulating_freq)            

            #outdata = np.add(carrier_signal,modulating_signal)

            outdata = carrier_signal

            global j 
            if j < 100:                    
                total_1 += list(outdata)
                total_1 += list([2])
                total_2 += list(carrier_signal)
                total_3 += list(modulating_signal)
                j += 1
    
            
            start_idx += frames

        try:     
            with sd.OutputStream(device=args.device, channels=1, callback=callback,
                                 samplerate=samplerate):
                print(samplerate)
                print('#' * 10)
                print('press Return to quit')
                print('#' * 10)
                input()
                plt.subplot(3,1,1)
                plt.plot(total_1)
                plt.subplot(3,1,2)
                plt.plot(total_2)
                plt.subplot(3,1,3)
                plt.plot(total_3)
                plt.show()
                input()

        except KeyboardInterrupt:
            parser.exit('')
        except Exception as e:
            parser.exit(type(e).__name__ + ': ' + str(e))

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text
        
if __name__ == "__main__":


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

    start_idx = 0
    test = SrFingerMount()
    rospy.spin()
    

 
  