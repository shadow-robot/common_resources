
import argparse
import sys

import rospy
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from sr_robot_msgs.msg import ShadowPST
from std_msgs.msg import Float32

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

global Am, Ac, fm, fc

fm = 0.1
Am = 0.2

Ac = 1
fc = 25

collection = []

def tactile_cb(data):        
    global thumb_press, Am_update, fm_update
    thumb_press = data.pressure[0]
    #thumb_max = 950
    #thumb_min = 330
    thumb_max = 200
    thumb_min = 0
    amp_max = 1
    amp_min = 1
    freq_min = 1
    freq_max = 1
    Am_update = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (amp_max - amp_min) + amp_min
    fm_update = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (freq_max - freq_min) + freq_min
    
try:
    samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

    rospy.init_node("sr_finger_mount_test")
    samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

    rospy.Subscriber("/rh/tactile", ShadowPST, tactile_cb)
    rospy.wait_for_message("/rh/tactile", ShadowPST)
    
    def callback(outdata, frames, time, status):
        if status:
            print(status, file=sys.stderr)

        global m_phase1, m_phase2, collection
        global Am, fm

        oldsignal = 0
        signal = np.zeros(frames)

        for i in range(frames):
            phaseInc1 = 2*np.pi*fm/samplerate
            phaseInc2 = 2*np.pi*fc/samplerate
            signal[i] = (Ac + Am * np.sin(m_phase1)) * np.sin(m_phase2)
            m_phase1 += phaseInc1
            m_phase2 += phaseInc2

            print(Ac, fc, Am, fm)
            if ((signal[i] <= 0) and (oldsignal >= 0) or (signal[i] >= 0) and (oldsignal <= 0)):
                fm = fm_update
                Am = Am_update
                
            oldsignal = signal[i]
            
        collection += list(signal)
        signal = signal.reshape(-1, 1)
        outdata[:] = signal

    with sd.OutputStream(device=args.device, channels=1, callback=callback,
                         samplerate=samplerate):
        print('#' * 80)
        print('press Return to quit')
        print('#' * 80)
        input()
        plt.plot(collection)
        plt.show()
except KeyboardInterrupt:
    parser.exit('')
except Exception as e:
    print(e)
    parser.exit(type(e).__name__ + ': ' + str(e))