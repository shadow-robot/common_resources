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

def tactile_cb(data):        
    global thumb_press, Am, fm
    thumb_press = data.pressure[0]
    #thumb_max = 950
    #thumb_min = 330
    thumb_max = 200
    thumb_min = 0

    amp_max = 1
    amp_min = 1
    freq_min = 1
    freq_max = 1
    Am = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (amp_max - amp_min) + amp_min
    fm = ((thumb_press - thumb_min) / (thumb_max - thumb_min)) * (freq_max - freq_min) + freq_min
    
try:
    samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

    rospy.init_node("sr_finger_mount_test")
    samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

    rospy.Subscriber("/rh/tactile", ShadowPST, tactile_cb)
    rospy.wait_for_message("/rh/tactile", ShadowPST)
    
    def callback(outdata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        global m_phase1, m_phase2, collection_1, collection_2
        global fm, fc, amplitude, oldsignal
        global amp, amphead, amphead_inc, amphead_dec
        global freq, freqhead, freqhead_inc, freqhead_dec
        newfrequency1 = freq[freqhead]
        #newamplitude = amp[amphead]
        
        #Am = 0.4
        Ac = 0.4
        #fm = 0
        
        signal = np.zeros(frames)
        signal_2 = np.zeros(frames)
        for i in range(frames):
            phaseInc1 = 2*np.pi*fm/samplerate
            phaseInc2 = 2*np.pi*fc/samplerate
            signal[i] = (Ac + Am * np.sin(m_phase1)) * np.sin(m_phase2)
            signal_2[i] = (Ac + Am * np.sin(m_phase1+5)) * np.sin(m_phase2-3)
            m_phase1 += phaseInc1
            m_phase2 += phaseInc2
            
            #print(i, amplitude, fm, newamplitude, newfrequency, signal[i], oldsignal)
            if ((signal[i] <= 0) and (oldsignal >= 0) or (signal[i] >= 0) and (oldsignal <= 0)):
                fc = newfrequency1
                #Am = newamplitude1
                
            oldsignal = signal[i]
            
        collection_1 += list(signal)
        collection_2 += list(signal_2)
        #collection += list([2])
        #t = (m_phase + np.arange(frames)) / samplerate
        #t = t.reshape(-1, 1)
        signal = signal.reshape(-1, 384)
        signal_2 = signal_2.reshape(-1, 384)
        #print(signal.shape)
        #print(outdata.shape)
        #signal = amplitude * np.sin(2 * np.pi * f1 * t)
        outdata[:,0] = signal
        outdata[:,1] = signal_2
        print(outdata.shape)
        #outdata[:,1] = signal_2


        #m_phase += frames
        
        #print(t)
        
        
        if amphead == 99:
            amphead_dec = True
            amphead_inc = False
        elif amphead == 0:
            amphead_inc = True
            amphead_dec = False
        if amphead_dec:
            amphead -= 1
        if amphead_inc:
            amphead += 1
            
        if freqhead == 99:
            freqhead_dec = True
            freqhead_inc = False
        elif freqhead == 0:
            freqhead_inc = True
            freqhead_dec = False
        if freqhead_dec:
            freqhead -= 1
        if freqhead_inc:
            freqhead += 1
        #print(frames)

    with sd.OutputStream(device=args.device, channels=2, callback=callback,
                         samplerate=samplerate):
        print('#' * 80)
        print('press Return to quit')
        print('#' * 80)
        input()
        plt.subplot(2,1,1)
        plt.plot(collection_1)
        plt.subplot(2,1,2)
        plt.plot(collection_2)
        plt.show()
except KeyboardInterrupt:
    parser.exit('')
except Exception as e:
    print(e)
    parser.exit(type(e).__name__ + ': ' + str(e))