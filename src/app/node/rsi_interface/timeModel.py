import time
import xml.etree.ElementTree as ET




class TimeModel:
	
	def __init__(self):
		self.timeDifference = None
		self.lastIPOC = None
	
	def getMillisecondTimestamp(self):
		return round((time.time()*1000))
	
	def setIPOC(self, ipoc):
		self.lastIPOC = ipoc
		self.timeDifference = ipoc - self.getMillisecondTimestamp()
		
		return self
	
	def getIPOC(self):
		return self.getMillisecondTimestamp()+self.timeDifference
	
	def setIpocByXmlString(self, data):
		xml = ET.fromstring(data)
		t = eval(xml.find("IPOC").text)
		
		self.setIPOC(t)