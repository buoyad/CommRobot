#! /usr/bin/env python
import serial, time, threading, Queue
from pymavlink import mavlinkv10 as mavlink
RxQueue = Queue.Queue()

class serialRead(threading.Thread):
	s = serial.Serial()
	port = None
	rate = 19200 # Probably

	def __init__(self, port):
		threading.Thread.__init__(self)
		self.port = port
		self.initSerial()

	def initSerial(self):
		self.s.port = self.port
		self.s.baudrate = self.rate
		self.s.timeout = 0.5
		self.s.open()
		self.s.flushInput()
		self.s.flushOutput()

	def run(self):
		# Only run AFTER modem is connected and turned on
		self.initSerial()
		self.configModem()
		while True:
			msg = self.s.readline()
			if (len(msg) != 0):
				RxQueue.put(msg)

	def configModem(self):  # From uModem.py, by Mike Puntolillo, 2011
		self.s.write('$CCCFG,REV,0\r\n');
		self.s.write('$CCCFG,ASD,0\r\n');
		self.s.write('$CCCFG,RXA,0\r\n');
		self.s.write('$CCCFG,RXD,1\r\n');
		self.s.write('$CCCFG,DTO,5\r\n');
		self.s.write('$CCCFG,AGN,200\r\n');
		self.s.write('$CCCFG,XST,0\r\n');
		self.s.write('$CCCFG,DOP,1\r\n');

class Beats(threading.Thread):
	
	def __init__(self):
		threading.Thread.__init__(self)

class WHOI(threading.Thread):
	s = None # Modem reader
	packetQueue = Queue.Queue() # Parsed MAVLINK to be sent to APM
	ret = 65536 # 2^16, to retain old channel value
	mav = None

	class fifo(object):
		def __init__(self):
			self.buf = []
		def write(self, data):
			self.buf += data
			return len(data)
		def read(self):
			return self.buf.pop(0)

	def __init__(self, port):
		threading.Thread.__init__(self)
		self.s = serialRead(port)

	def run(self):
		self.s.start()
		MAVBuffer = self.fifo()
		mav = mavlink.MAVLink(MAVBuffer)
		mav.srcSystem = 255
		mav.srcComponent = 190
		while True:
			if not RxQueue.empty():
				msg = self.RxQueue.get()
				if msg.startswith('$CAMUA'):
					v = msg.split(',')
					data = (v[3].split('*'))[0]
					data = int(data, 16)
					bind = bin(data)
					cmd = bnd[2:5]
					val = bnd[5:]
					# Pack appropriate MAVLINK message, send along.
					if cmd == '000': # RC Override Channel 1
						ch_raw = int(val, 2) + 1000
						mav_pack = mav.MAVLink_rc_channels_override_message(100, 100, ch_raw, ret, ret, ret, ret, ret, ret, ret)
						packetQueue.put(mav_pack.pack(mav))
					elif cmd == '001':

					elif cmd == '010':

					elif cmd == '011':

					elif cmd == '100':

					elif cmd == '101':

					elif cmd == '110':

					elif cmd == '111':

	def receive(self):
		msg = RxQueue.get()
		return msg
