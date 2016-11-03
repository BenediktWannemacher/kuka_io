#!/usr/bin/env python
import rospy
from app.core import util
from app.node.rsi_interface.controller import Controller



def main(argv=None):
	rospy.init_node('kuka_rsi_interface', anonymous=True)
	util.logBeginningInfo("rsi_interface")
	
	try:
		ip,port = getArgs()
	except:
		rospy.logfatal("Invalid port value!")
		return
	
	controller = Controller(ip, port)
	
	loopFreq = 120
	
	rate = rospy.Rate(loopFreq)
	rospy.loginfo("Start the main loop. Frequency: "+str(loopFreq)+" Interval: "+str(1000/loopFreq)+"ms")
	
	while not rospy.is_shutdown():
		controller.loop()
		rate.sleep()
	
	rospy.spin()
#eof



def getArgs():
	ip = rospy.get_param('~ip', "")
	port = rospy.get_param('~port', None)
	
	if(port is None):
		raise ValueError("No tag has been defined!")
	
	return ip,port
#eof



if __name__ == '__main__':
	main()