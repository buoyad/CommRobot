#! /usr/bin/env python
import serial, time, sys, os, threading
from dronekit import connect, VehicleMode
from pymavlink import mavlinkv10 as mavlink

#ground = connect('/dev/ttyUSB1', baud=57600, wait_ready=true)
apm = serial.Serial()
apm.port = '/dev/tty60'
apm.baudrate = 57600
radio = serial.Serial()
radio.baudrate = 57600
radio.port = '/dev/ttyAMA0'
#@vehicle.on_message()
#def listener(self, name, message):
#    print message

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

f = fifo()
mav = mavlink.MAVLink(f)

class rfLink (threading.Thread):
	src = None
	dest = None
	name = None
	def __init__(self, src, dest, name, **kwds):
		threading.Thread.__init__(self)
		self.src = src
		self.dest = dest
		self.name = name
		try:
			if(kwds['daemon']): 
				self.daemon = True
			else:
				self.daemon = False
		except KeyError:
			pass
	def run(self):
		self.src.open()
		self.dest.open()
		self.src.flushInput()
		self.dest.flushOutput()
		self.handover()

	def handover(self):
		print 'Started ' + self.name
		while True:
			if (self.src.inWaiting()>0):
				g = mav.parse_buffer(self.src.read())
				if (g!=None):
					self.dest.write(str(bytearray(f)))
				
rTa = rfLink(radio, apm, 'Radio to APM')
aTr = rfLink(apm, radio, 'APM to Radio', daemon=True)
rTa.start()
aTr.start()

