#! /usr/bin/env python
import serial, threading
#from dronekit import connect, VehicleMode
from pymavlink import mavlinkv10 as mavlink


class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class rfLink (threading.Thread):
	src = None
	dest = None
	name = None
	mav = None
	def __init__(self, src, dest, name, **kwds):
		threading.Thread.__init__(self)
		self.src = src
		self.dest = dest
		self.name = name
		self.f = fifo()
		self.mav = mavlink.MAVLink(self.f)
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
			try:
				g = self.mav.parse_buffer(self.src.read())
			except mavlink.MAVError:
				pass
			if (g!=None):
				#self.dest.write(str(bytearray(self.f)))
				print(g)
                                print(g[0].type)
                                print(g[0].autopilot)
                                print(g[0].base_mode)
                                print(g[0].custom_mode)
                                print(g[0].system_status)
                                print(g[0].mavlink_version)
				self.src.write(str(bytearray(self.f.buf)))
				