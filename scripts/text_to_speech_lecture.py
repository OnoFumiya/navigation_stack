#!/usr/bin/env python3
# coding: utf-8
import rospy
from text_to_speech.srv import TextToSpeech
# from std_msgs.msg import String


def speech(word):
	rospy.wait_for_service('/speech_word')
	try:
		text_to_speech = rospy.ServiceProxy('/speech_word', TextToSpeech)
		print(word)
		resp = text_to_speech( word )
	except rospy.ServiceException as e:
		print ("Service call failed: {0}".format(e))


if __name__ == '__main__':
    rospy.init_node('mapping_plot')
    w = 'Hello. I am cheerful college student'
    speech(w)
    rospy.spin()