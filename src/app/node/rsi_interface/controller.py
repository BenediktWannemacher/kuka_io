import rospy
from app.core import util
from std_msgs.msg._String import String
import xml.etree.ElementTree as ET
from app.node.rsi_interface.timeModel import TimeModel
from thread import start_new_thread
import socket




class Controller:
	
	
	def __init__(self, recvIp, recvPort):
		self.recvIp = recvIp
		self.recvPort = recvPort
		
		self.kukaIp = None
		self.kukaPort = None
		
		self.msgTemplateXml = None
		self.timeModel = TimeModel()
		
		self.msgFromKukaPublisher = rospy.Publisher(util.topicName("rsi_interface", "msg_from_kuka"), String, queue_size=1)
		self.msgToKukaPublisher = rospy.Publisher(util.topicName("rsi_interface", "msg_to_kuka"), String, queue_size=1)
		
		self.msgToKukaSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		rospy.Subscriber(util.topicName("rsi_interface", "msg_template"), String, self.msgTemplateCb)
		rospy.logwarn("First message template needs to be published before starting rsi...")
		
		
		start_new_thread(self.recvFromKukaThread, ())
		rospy.logwarn("recvFromKukaThread is running! Waiting for handshake message...")
	#eof
	
	
	
	def msgTemplateCb(self, msg):
		try:
			firstMsg = False
			if(self.msgTemplateXml is None):
				firstMsg = True
				
			self.msgTemplateXml = ET.ElementTree(ET.fromstring(msg.data))
			
			if(firstMsg):
				rospy.logwarn("First message template recieved!")
		except:
			rospy.logerr("Invalid Message! Can not parse as XML")
	#eof
	
	
	
	def recvFromKukaThread(self):
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.bind((self.recvIp, self.recvPort))
		
		rospy.logwarn("Start listening. IP: "+str(self.recvIp)+" PORT: "+str(self.recvPort))
	
		while(True):
			data, addr = sock.recvfrom(1024)
		
			if(self.kukaIp is None or self.kukaPort is None):
				self.kukaIp = addr[0]
				self.kukaPort = addr[1]
				rospy.logwarn("handshake message recieved. IP: "+str(self.kukaIp)+" ,PORT: "+str(self.kukaPort))
		
			self.msgFromKukaPublisher.publish(data)
			self.timeModel.setIpocByXmlString(data)
	#eof
	
	
	
	def loop(self):
		if(self.msgTemplateXml is not None and self.kukaIp is not None and self.kukaPort is not None):
			msgStr = ET.tostring(self.msgTemplateXml.getroot())
			
			self.msgTemplateXml.find("IPOC").text = str(self.timeModel.getIPOC())
			self.msgToKukaSock.sendto(msgStr, (self.kukaIp, self.kukaPort))
			self.msgToKukaPublisher.publish(msgStr)
	#eof
			