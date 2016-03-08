#! /usr/bin/env python
import serial, time, threading


class WHOI(threading.Thread):
	s = serial.Serial() # Modem
	port = none
	rate = 19200 # Probably
	
	def __init__(self, port):
		threading.Thread.__init__(self)
		self.port = port
		self.initSerial()
		self.configModem()

	def initSerial(self):
		self.s.port = self.port
		self.s.baudrate = self.rate
		self.s.open()
		self.s.flushInput()
		self.s.flushOutput()

	def configModem(self):  # From uModem.py, by Mike Puntolillo, 2011
		self.s.write('$CCCFG,REV,0\r\n');
                self.s.write('$CCCFG,ASD,0\r\n');
                self.s.write('$CCCFG,RXA,0\r\n');
                self.s.write('$CCCFG,RXD,1\r\n');
                self.s.write('$CCCFG,DTO,5\r\n');
                self.s.write('$CCCFG,AGN,200\r\n');
                self.s.write('$CCCFG,XST,0\r\n');
                self.s.write('$CCCFG,DOP,1\r\n');
