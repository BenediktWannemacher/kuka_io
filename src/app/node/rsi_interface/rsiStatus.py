from kuka_io.msg._Status import Status



class RsiStatus:
	
	def __init__(self, num, level, title, desc):
		self.num = num
		self.msg = Status()
		self.msg.id = num
		self.msg.level = level
		self.msg.title = title
		self.msg.description = desc
	
	
	
	def isSame(self, obj):
		return self.num == obj.num
#eoc




WAITING_FOR_TEMPLATE_AND_HANDSHAKE = RsiStatus(0, 'INFO', 'Waiting...', 'Waiting for template and handshake message.')
WAITING_FOR_TEMPLATE = RsiStatus(1, 'ERROR', 'Waiting For Template', 'Handshake message arrived before first message template! Can not start RSI.')
WAITING_FOR_HANDSHAKE = RsiStatus(2, 'INFO', 'Waiting For Handshake', 'First message template arrived. Waiting for handshake message to start RSI')
RSI_RUNNING = RsiStatus(3, 'SUCCESS', 'Running', 'RSI is running!')
RSI_CANCELLED = RsiStatus(4, 'WARNING', 'RSI Cancelled', 'No package has been recieved. Probably the RSI has been turned off')
RSI_TIMEOUT = RsiStatus(5, 'ERROR', 'RSI Timeout', 'Delayed packages has exceed the limit. RSI has stopped working')