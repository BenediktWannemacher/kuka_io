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
	
	rate = rospy.Rate(0.75)
	
	controller = Controller(ip, port)
	
	while(not rospy.is_shutdown()):
		controller.publishStatus()
		rate.sleep()
#eof



def getArgs():
	ip = rospy.get_param('~ip', "")
	port = rospy.get_param('~port', None)
	
	if(port is None):
		raise ValueError(rospy.get_namespace() + "No tag has been defined!")
	
	return ip,port
#eof



if __name__ == '__main__':
	main()