#!/usr/bin/env python
import rospy
from app.core import util
from app.node.extract_a6.controller import Controller



def main(argv=None):
	rospy.init_node('kuka_extract_a6', anonymous=True)
	util.logBeginningInfo("extract_a6")
	
	Controller(getArgs())
	
	rospy.spin()
#eof



def getArgs():
	ignore = rospy.get_param('~ignore', None)	
	return ignore
#eof



if __name__ == '__main__':
	main()
