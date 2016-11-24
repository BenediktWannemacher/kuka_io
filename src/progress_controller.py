#!/usr/bin/env python
import rospy
from app.core import util
from app.node.progress_controller.controller import Controller



def main(argv=None):
	rospy.init_node('progress_controller', anonymous=True)
	util.logBeginningInfo("progress_controller")
	
	ignore,feedbackTopic, effectorTopic = getArgs()
	
	if(feedbackTopic is not None and effectorTopic is not None):
		Controller(feedbackTopic, effectorTopic, ignore)
		rospy.spin()
	else:
		util.logErr("feedbackTopic has to be specified...")
#eof



def getArgs():
	ignore = rospy.get_param('~ignore', None)
	feedbackTopic = rospy.get_param('~feedbackTopic', None)
	effectorTopic = rospy.get_param('~effectorTopic', None)
		
	return ignore,feedbackTopic, effectorTopic
#eof



if __name__ == '__main__':
	main()
