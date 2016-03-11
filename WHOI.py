#! /usr/bin/env python
import serial, time, threading, Queue
RxQueue = Queue.Queue()

class serialRead(threading.Thread):
	s = serial.Serial()
	port = None
	rate = 19200 # Probably

	def __init__(self, port, queue):
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



class WHOI(threading.Thread):
	s = None # Modem reader

	def __init__(self, port):
		threading.Thread.__init__(self)
		self.s = serialRead(port, RxQueue)

	def run(self):
		self.s.start()

	def receive(self):
		msg = RxQueue.get()
		return msg
