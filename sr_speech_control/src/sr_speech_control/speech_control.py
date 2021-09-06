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

import rospy
import rospkg
import time
import speech_recognition as sr
from difflib import get_close_matches
from std_msgs.msg import String
import yaml
from builtins import input

from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
from pydub.playback import play


class SpeechControl(object):
    def __init__(self, trigger_word, command_words, command_topic='sr_speech_control',
                 similar_words_dict_path=None, non_speaking_duration=0.2, pause_threshold=0.2):
        self.trigger_word = trigger_word
        self.command_words = command_words
        self.recognizer = sr.Recognizer()
        self.command_publisher = rospy.Publisher(command_topic, String, queue_size=1)
        self.command_to_be_executed = None
        self.similar_words_dict = {}

        if similar_words_dict_path:
            self.parse_similar_words_dict(similar_words_dict_path)
        self._init_recognizer(non_speaking_duration, pause_threshold)
        self._stop_listening = self.recognizer.listen_in_background(self.microphone, self._recognizer_callback)

    def parse_similar_words_dict(self, path_name):
        with open(path_name, 'r') as stream:
            self.similar_words_dict = yaml.safe_load(stream)

    def _init_recognizer(self, non_speaking_duration, pause_threshold):
        for idx, mic in enumerate(sr.Microphone.list_microphone_names()):
            rospy.loginfo('{}: {}'.format(idx, mic))

        while not rospy.is_shutdown():
            try:
                idx = input("Choose one of the microphones from the list above. "
                            "Type the index or leave empty for default microphone, than press [RETURN]\n")
                if not idx:
                    self.microphone = sr.Microphone()
                else:
                    self.microphone = sr.Microphone(device_index=int(idx))
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source)
                    break
            except OSError:
                rospy.logwarn("Wrong microphone. Try again.")

        self.recognizer.non_speaking_duration = non_speaking_duration
        self.recognizer.pause_threshold = pause_threshold

    def _recognizer_callback(self, recognizer, audio):
        try:
            result = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            rospy.logwarn("Could not request results from Google Speech Recognition service: {}".format(e))
            return

        result = [str(x).lower() for x in result.split(' ')]

        if self._filter_word(result[0], self.trigger_word) == self.trigger_word:
            command = self._filter_word(''.join(result[1:]), self.command_words)
            if command in self.command_words:
                self.command_to_be_executed = command

    def _filter_word(self, word, dictionary, offset=0.5):
        if word in self.similar_words_dict:
            word = self.similar_words_dict[word]

        result = get_close_matches(word, dictionary, 1, offset)
        if not result:
            return word
        return result[0]


    def _confirm_voice_command(self, command):
        language = 'en'
        tts = gTTS(text=command, lang=language, slow=False)
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)
        audio = AudioSegment.from_file(fp, format="mp3")
        play(audio)

    def run(self):
        rospy.loginfo("Started speech control. Trigger word: {}".format(self.trigger_word))
        while not rospy.is_shutdown():
            if self.command_to_be_executed:
                rospy.loginfo("Executing: {}.".format(self.command_to_be_executed))
                self.command_publisher.publish(self.command_to_be_executed)
                self._confirm_voice_command(self.command_words[self.command_to_be_executed])
                self.command_to_be_executed = None
        self._stop_listening(wait_for_stop=False)


if __name__ == "__main__":

    rospy.init_node('example_speech_control', anonymous=True)

    trigger_word = "shadow"
    command_words_and_feedback = {"grasp":"grasped", "release":"released", "disable":"disabled", "enable":"enabled", "engage":"engaged", "open":"opened"}
    similar_words_dict_path = rospkg.RosPack().get_path('sr_speech_control') + '/config/similar_words_dict.yaml'

    sc = SpeechControl(trigger_word, command_words_and_feedback, similar_words_dict_path=similar_words_dict_path)
    sc.run()

