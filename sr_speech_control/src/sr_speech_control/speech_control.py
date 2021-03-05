#!/usr/bin/env python

import rospy
import time
import speech_recognition as sr
from difflib import get_close_matches
from std_msgs.msg import String

class SpeechControl(object):
    def __init__(self, trigger_word, command_words, command_topic='speech_control',
                 non_speaking_duration=0.1, pause_threshold=0.2):
        self.trigger_word = trigger_word
        self.command_words = command_words
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.command_publisher = rospy.Publisher(command_topic, String, queue_size=1)
        self.command_to_be_executed = None

        self._init_recognizer(non_speaking_duration, pause_threshold)
        self._stop_listening = self.recognizer.listen_in_background(self.microphone, self._recognizer_callback)

    def _init_recognizer(self, non_speaking_duration, pause_threshold):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
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
            command = self._filter_word(result[1], self.command_words)
            if command in self.command_words:
                self.command_to_be_executed = command

    def _filter_word(self, word, dictionary, offset = 0.5):
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
        self._stop_listening(wait_for_stop=False)


if __name__ == "__main__":
    rospy.init_node('example_speech_control', anonymous=True)

    trigger_word = "shadow"
    command_words = ["grasp", "release", "disable", "enable"]
    sc = SpeechControl(trigger_word, command_words)
    sc.run()
