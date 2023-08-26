#!/usr/bin/env python3

# Copyright 2021-2022 Shadow Robot Company Ltd.
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

import time
from difflib import get_close_matches
from threading import Thread
import subprocess
import wave
import speech_recognition as sr
from std_msgs.msg import String
import yaml
from gtts import gTTS
import pyaudio
import rospy
import rospkg


def write_mp3(tts, proc):
    tts.write_to_fp(proc.stdin)
    proc.stdin.close()


def text_to_speech(text, device_name):
    pyaud = pyaudio.PyAudio()
    device_index = None
    sample_rate = None
    for index in range(0, pyaud.get_device_count()):
        device_info = pyaud.get_device_info_by_index(index)
        if device_name == device_info['name']:
            device_index = index
            sample_rate = device_info['defaultSampleRate']
            break
    tts = gTTS(text=text, lang='en', slow=False)
    # Use ffmpeg top convert mp3 to wav
    with subprocess.Popen(['ffmpeg', '-loglevel', 'quiet', '-i', 'pipe:', '-ar', str(sample_rate), '-ac',
                           '1', '-f', 'wav', 'pipe:'], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as proc:
        # Piping mp3 to ffmepg standard input must be in a separate thread because pipe
        # buffers are relatively small and process will block before any output is received
        thread = Thread(target=write_mp3, args=(tts, proc, ))
        thread.start()
        # Read wav from ffmpeg standard output
        waveform = wave.open(proc.stdout, 'rb')
        stream = pyaud.open(format=pyaud.get_format_from_width(waveform.getsampwidth()),
                            channels=waveform.getnchannels(),
                            rate=waveform.getframerate(),
                            output_device_index=device_index,
                            output=True)
        stream.write(waveform.readframes(waveform.getnframes()))
        stream.stop_stream()
        stream.close()
        pyaud.terminate()
        thread.join()


class SpeechControl:
    def __init__(self, trigger_word, command_words, command_topic='/sr_speech_control/speech_to_text',
                 similar_words_dict_path=None, non_speaking_duration=0.2, pause_threshold=0.2):

        self.trigger_word = trigger_word
        self.command_words = command_words
        self.recognizer = sr.Recognizer()
        self.command_publisher = rospy.Publisher(command_topic, String, queue_size=1)
        self.command_to_be_executed = None
        self.similar_words_dict = {}
        self.microphone = ''
        self.microphone_subscriber = rospy.Subscriber('/sr_speech_control/audio_device/input',
                                                      String,
                                                      self.microphone_callback)
        self.speaker = ''
        self.speaker_subscriber = rospy.Subscriber('/sr_speech_control/audio_device/output',
                                                   String,
                                                   self.speaker_callback)
        self.tts_subscriber = rospy.Subscriber('/sr_speech_control/text_to_speech',
                                               String,
                                               self.text_to_speech_callback)

        if similar_words_dict_path:
            self.parse_similar_words_dict(similar_words_dict_path)
        self._init_recognizer(non_speaking_duration, pause_threshold)
        self._stop_listening = None

    def microphone_callback(self, message):
        self.microphone = message.data
        # Stop listening on previously select microphone
        if self._stop_listening:
            self._stop_listening(wait_for_stop=True)
        # If new microphone is not empty (speech recognition not disabled) start listening
        if self.microphone != '':
            for index, name in enumerate(sr.Microphone.list_microphone_names()):
                if name == self.microphone:
                    microphone = sr.Microphone(device_index=int(index))
                    with microphone as source:
                        self.recognizer.adjust_for_ambient_noise(source)
                    rospy.loginfo('Using microphone {} for speech recognition'.format(name))
                    self._stop_listening = self.recognizer.listen_in_background(microphone,
                                                                                self._recognizer_callback)
                    break

    def speaker_callback(self, message):
        self.speaker = message.data

    def text_to_speech_callback(self, message):
        if self.speaker != '':
            text_to_speech(message.data, self.speaker)

    def parse_similar_words_dict(self, path_name):
        with open(path_name, 'r', encoding='UTF-8') as stream:
            self.similar_words_dict = yaml.safe_load(stream)

    def _init_recognizer(self, non_speaking_duration, pause_threshold):
        self.recognizer.non_speaking_duration = non_speaking_duration
        self.recognizer.pause_threshold = pause_threshold

    def _recognizer_callback(self, recognizer, audio):
        try:
            result = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return
        except sr.RequestError as exception:
            rospy.logwarn(f"Could not request results from Google Speech Recognition service: {exception}")
            return

        result = [str(result_part).lower() for result_part in result.split(' ')]
        rospy.loginfo("Received: {}".format(result))

        if self._filter_word(result[0], self.trigger_word) == self.trigger_word:
            command = self._filter_word(''.join(result[1:]), self.command_words)
            rospy.loginfo("Understood as : {}".format(command))
            if command in self.command_words:
                self.command_to_be_executed = command

    def _filter_word(self, word, dictionary, offset=0.5):
        if word in self.similar_words_dict:
            word = self.similar_words_dict[word]

        result = get_close_matches(word, dictionary, 1, offset)
        if not result:
            return word
        return result[0]

    def run(self):
        rospy.loginfo("Started speech control. Trigger word: {}".format(self.trigger_word))
        while not rospy.is_shutdown():
            if self.command_to_be_executed:
                rospy.loginfo("Executing: {}.".format(self.command_to_be_executed))
                self.command_publisher.publish(self.command_to_be_executed)
                self.command_to_be_executed = None
            time.sleep(0.01)
        if self._stop_listening:
            self._stop_listening(wait_for_stop=False)


if __name__ == "__main__":

    rospy.init_node('example_speech_control', anonymous=True)

    trigger_word_arg = "shadow"
    command_words_and_feedback_arg = {"grasp": "grasped", "release": "released", "disable": "disabled",
                                      "enable": "enabled", "engage": "engaged", "disengage": "disengaged",
                                      "open": "opened"}
    similar_words_dict_path_arg = rospkg.RosPack().get_path('sr_speech_control') + '/config/similar_words_dict.yaml'

    sc = SpeechControl(trigger_word_arg, command_words_and_feedback_arg,
                       similar_words_dict_path=similar_words_dict_path_arg)
    sc.run()
