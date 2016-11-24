import rospy
from app.core import util
from std_msgs.msg._String import String
import xml.etree.ElementTree as ET
from kuka_io.msg._A6 import A6




class Controller:
	
	
	def __init__(self, ignore=None):
		
		self.ignore = ignore
		self.ignoreCounter = 0
		
		self.valuePublisher = rospy.Publisher(util.topicName("extract_a6", "value"), A6, queue_size=1)
		
		if(self.ignore is not None):
			self.valueWithDelayPublisher = rospy.Publisher(util.topicName("extract_a6", "value_with_delay"), A6, queue_size=1)
		
		rospy.Subscriber(util.topicName("rsi_interface", "msg_from_kuka"), String, self.msgFromKukaCb)
	#eof
	
	
	
	def msgFromKukaCb(self, msg):
		try:
			data = A6()
			val = ET.fromstring(msg.data).find("ASPos")
	
			data.A1 = float(val.get('A1'))
			data.A2 = float(val.get('A2'))
			data.A3 = float(val.get('A3'))
			data.A4 = float(val.get('A4'))
			data.A5 = float(val.get('A5'))
			data.A6 = float(val.get('A6'))
			
			self.valuePublisher.publish(data)
			
			if(self.ignore is not None):
				if(self.ignoreCounter == 0):
					self.valueWithDelayPublisher.publish(data)
				
				self.ignoreCounter += 1
				self.ignoreCounter %= self.ignore
		except:
			util.logErr("Parsing Error! This message will be ignored.");
	#eof
#eoc