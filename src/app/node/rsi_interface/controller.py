import rospy
from app.core import util
from std_msgs.msg._String import String
import xml.etree.ElementTree as ET
from thread import start_new_thread
import socket
import re




class Controller:
	
	
	def __init__(self, recvIp, recvPort):
		self.recvIp = recvIp
		self.recvPort = recvPort
		
		self.kukaIp = None
		self.kukaPort = None
		
		self.msgTemplateString = None
		self.lastIPOC = None #includes xml tags <IPOC>
		self.regex = re.compile(r"<IPOC>(\d+?)<\/IPOC>")
		
		self.msgFromKukaPublisher = rospy.Publisher(util.topicName("rsi_interface", "msg_from_kuka"), String, queue_size=1)
		self.msgToKukaPublisher = rospy.Publisher(util.topicName("rsi_interface", "msg_to_kuka"), String, queue_size=1)
		
		self.msgToKukaSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		rospy.Subscriber(util.topicName("rsi_interface", "msg_template"), String, self.msgTemplateCb)
		util.logWarn("First message template needs to be published before starting rsi...")
		
		start_new_thread(self.recvFromKukaThread, ())
		util.logWarn("recvFromKukaThread is running! Waiting for handshake message...")
	#eof
	
	
	
	def msgTemplateCb(self, msg):
		try:
			if(ET.fromstring(msg.data).find("IPOC") is None):
				util.logErr("Missing IPOC! IPOC tag has to be included in message template. Can not set message template by this message...")
				util.logErr(msg.data)
				return
			
			firstMsg = False
			if(self.msgTemplateString is None):
				firstMsg = True
			
			self.msgTemplateString = msg.data
			
			if(firstMsg):
				util.logWarn("First message template recieved!")
		except:
			util.logErr("Invalid xml structure! Can not set message template by this message...")
			util.logErr(msg.data)
	#eof
	
	
	
	def recvFromKukaThread(self):
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.bind((self.recvIp, self.recvPort))
		
		util.logWarn("Start listening. IP: "+str(self.recvIp)+" PORT: "+str(self.recvPort))
	
		while(True):
			data, addr = sock.recvfrom(1024)
		
			if(self.kukaIp is None or self.kukaPort is None):
				self.kukaIp = addr[0]
				self.kukaPort = addr[1]
				util.logWarn("Handshake message recieved. IP: "+str(self.kukaIp)+" ,PORT: "+str(self.kukaPort))
			
			self.lastIPOC = self.regex.search(data).group(0)
			self.sendToKuka()
			self.msgFromKukaPublisher.publish(data)
		#eo while
	#eof
	
	
	
	def sendToKuka(self):
		if(self.msgTemplateString is not None and self.kukaIp is not None and self.kukaPort is not None):
			
			msgStr = self.regex.sub(self.lastIPOC, self.msgTemplateString)
			
			self.msgToKukaSock.sendto(msgStr, (self.kukaIp, self.kukaPort))
			self.msgToKukaPublisher.publish(msgStr)
		else:
			util.logErr("First message template or first handshake message has not arrived yet! Can not send data/start rsi")
	#eof
	
	
#eoc
			