#!/usr/bin/env python

import rospy
import pyaudio
import speech_recognition as sr
import multiprocessing
import threading
from std_msgs.msg import String

class AudioInterface:
    def __init__(self, wordlist) -> None:
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.stopper_func = None
        self.wordlist = wordlist

        self.publisher = rospy.Publisher("interface_commands", String)

        with self.mic as source: 
            self.recognizer.adjust_for_ambient_noise(source)

    def listen(self):
        self.stopper_func = self.recognizer.listen_in_background(self.mic, self.monitor_hotwords, 2)

    def monitor_hotwords(self, _, audio):
        try:
            words = self.recognizer.recognize_sphinx(audio)
            for w in words.split():
                print(w)
                if w in self.wordlist:
                    self.publish_hotword(w)
        except sr.UnknownValueError:
            print("I'm sorry, I didn't quite get that")


    def publish_hotword(self, word):
        new_msg = String()
        new_msg.data = word
        self.publisher.publish(new_msg)


if __name__=="__main__":
    rospy.init_node("audio_interface", anonymous=True)
    keywords = ["start", "bite", "stop", "I'm", "done"]
    interface = AudioInterface(keywords)
    interface.listen()
    rospy.spin()