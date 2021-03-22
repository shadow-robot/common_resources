#!/usr/bin/env python

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

import rospy
import argparse
import speech_recognition as sr
from std_msgs.msg import String


class SpeechControl(object):
    def __init__(self, prefer_microphone, trigger_word, command_topic='sr_speech_control'):
        self.microphone = sr.Microphone()
        for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
            if prefer_microphone in microphone_name:
                self.microphone = sr.Microphone(device_index=i)
                rospy.loginfo("Using preferred microphone: {}".format(microphone_name))
                break
        self.trigger_word = trigger_word
        self.recognizer = sr.Recognizer()
        self.command_publisher = rospy.Publisher(command_topic, String, queue_size=1)
        self._stop_listening = self.recognizer.listen_in_background(self.microphone, self._recognizer_callback)

    def _recognizer_callback(self, recognizer, audio):
        try:
            result = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            rospy.logwarn("Could not request results from Google Speech Recognition service: {}".format(e))
            return

        result = result.lower()
        if result.startswith(self.trigger_word) and len(result) > len(self.trigger_word) + 1:
            self.command_publisher.publish(result[len(self.trigger_word) + 1:])

    def run(self):
        rospy.loginfo("Started speech control. Trigger word: {}".format(self.trigger_word))
        rospy.spin()
        self._stop_listening(wait_for_stop=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Publish control commands using voice recognition.')
    parser.add_argument('--trigger-word', default='shadow')
    parser.add_argument('--prefer-microphone', default='pulse')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('sr_speech_control', anonymous=True)

    sc = SpeechControl(args.prefer_microphone, args.trigger_word)
    sc.run()
