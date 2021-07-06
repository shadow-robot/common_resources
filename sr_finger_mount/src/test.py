#!/usr/bin/env python3
"""Play a sine signal."""
import argparse
import sys
import rospy

import numpy as np
import sounddevice as sd
from sr_robot_msgs.msg import ShadowPST
from std_msgs.msg import Float32

global thumb_press, amp, freq
global carrier, modulating

carrier_freq = 20
carrier_amp = 0.1

carrier_signal = np.sin()

class SrFingerMount():
    def tactile_cb(self, data):        
        global thumb_press, amp, freq
        thumb_press = data.pressure[0]
        #thumb_max = 950
        #thumb_min = 330
        thumb_max = 200
        thumb_min = 0
        amp_max = 1.5
        amp_min = 0.2
        freq_min = 5
        freq_max = 50
        amp = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (amp_max - amp_min) + amp_min
        freq = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (freq_max - freq_min) + freq_min

    def __init__(self):

        rospy.init_node("sr_finger_mount_test")
        samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

        rospy.Subscriber("/rh/tactile", ShadowPST, self.tactile_cb)
        rospy.wait_for_message("/rh/tactile", ShadowPST)
       
        #self.pub = rospy.Publisher('/sine_debug', Float32)

        def callback(outdata, frames, time, status):
            if status:
                print(status, file=sys.stderr)

            global start_idx, amp, freq

            t = (start_idx + np.arange(frames)) / samplerate
            t = t.reshape(-1, 1)
            outdata[:] = amp * np.sin(2 * np.pi * freq * t)
            '''
            for i in outdata:
                self.pub.publish(i)
            '''
            start_idx += frames

        try:     
            with sd.OutputStream(device=args.device, channels=1, callback=callback,
                                samplerate=samplerate):
                print(samplerate)
                print('#' * 10)
                print('press Return to quit')
                print('#' * 10)
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
    

 
  