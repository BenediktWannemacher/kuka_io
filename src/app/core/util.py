import rospy
def topicName(pkgName, topicName):
	"""
	generate name of the topic based on the convention
	:param pkgName: name of the package without any slash
	:param topicName: name of the package without any slash at the begining or end
	"""
	return "kuka_io/"+pkgName+"/"+topicName
#eof




def logBeginningInfo(pkgName):
	print "======= kuka_io / "+pkgName+" ======="
#eof




def getStampedTime(obj):
	return obj.header.stamp.secs + (obj.header.stamp.nsecs/1000000000.0)
#eof




def logInfo(msg):
	o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	rospy.loginfo(o)
#eof




def logWarn(msg):
	o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	rospy.logwarn(o)
#eof




def logErr(msg):
	o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	rospy.logerr(o)