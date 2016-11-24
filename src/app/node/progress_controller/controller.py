import rospy
from app.core import util
from std_msgs.msg._String import String
import xml.etree.ElementTree as ET




class Controller:
	
	
	def __init__(self, feedbackTopic, effectorTopic, ignore=None):
		
		self.ignore = ignore
		self.ignoreCounter = 0
		
		self.feedbackTopic = feedbackTopic
		self.effectorTopic = effectorTopic
		
		self.feedbackPublisher = rospy.Publisher(self.feedbackTopic, String, queue_size=1)
		self.effectorPublisher = rospy.Publisher(self.effectorTopic, String, queue_size=1)
		
		rospy.Subscriber(util.topicName("rsi_interface", "msg_from_kuka"), String, self.msgFromKukaCb)
	#eof
	
	
	
	def msgFromKukaCb(self, msg):
		try:
			fdata = String()
			edata = String()
			
			edata.data = 's'
			
			tid = ET.fromstring(msg.data).find("ReturnTaskID").text
			prg = ET.fromstring(msg.data).find("ReturnTaskProgress").text

			fdata.data = tid+","+prg
			
			if(self.ignore is None):
				self.feedbackPublisher.publish(fdata)
				
				if(float(prg) == 50):
					self.effectorPublisher.publish(edata)
			else:
				if(self.ignoreCounter == 0):
					self.feedbackPublisher.publish(fdata)
					
					if(float(prg) == 50):
						self.effectorPublisher.publish(edata)
				
				self.ignoreCounter += 1
				self.ignoreCounter %= self.ignore
		except:
			util.logErr("Parsing Error! This message will be ignored.");
	#eof
#eoc