#! /usr/bin/env python
import serial, time, threading, Queue
from pymavlink import mavlinkv10 as mavlink
from dronekit import connect, VehicleMode
RxQueue = Queue.Queue()


class fifo(object):
	def __init__(self):
		self.buf = []
	def write(self, data):
		self.buf += data
		return len(data)
	def read(self):
		return self.buf.pop(0)


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
		print('Modem configured\n')

class Beats(threading.Thread):
	mav = None
	s = None

	def __init__(self):
		threading.Thread.__init__(self)
		MAVBuffer = fifo()
		self.mav = mavlink.MAVLink(MAVBuffer)
		self.mav.srcSystem = 255
		self.mav.srcComponent = 190
		self.s = serial.Serial()
		self.s.port = '/dev/ttyUSB0' # Check validity
		self.s.baudrate = 57600

	def run(self):
		self.s.open
		self.s.flushOutput()
		self.s.flushInput()
		print('Starting listener...\n')
		while True:
			# Type, APM, base_mode, custom_mode, system_status, mavlink version
			msg = self.mac.MAVLink_heartbeat_message(6, 3, x, y, z, 10) # x = base_mode, y = custom_mode, z = system_status
			self.s.write(str((bytearray(msg.pack()))))
			time.sleep(30)


class WHOI(threading.Thread):
	s = None # Modem reader
	packetQueue = Queue.Queue() # Parsed MAVLINK to be sent to APM
	ret = 65536 # 2^16, to retain old channel value
	mav = None
	mavsink = None

	def __init__(self, port, mavsink):
		threading.Thread.__init__(self)
		self.s = serialRead(port)
		MAVBuffer = fifo()
		self.mav = mavlink.MAVLink(MAVBuffer)
		self.mav.srcSystem = 255
		self.mav.srcComponent = 190
		self.mavsink = mavsink

	def run(self):
		self.s.start()
		#hBeats = Beats()
		#hBeats.start()
		while True:
			if not RxQueue.empty():
				msg = self.receive()
				print(msg);
				time.sleep(.05);
				#self.mavsink.send_mavlink(self.mavsink.message_factory.rc_channel_override_encode(
				if msg.startswith('$CAMUA'):		# Mini packet received
					v = msg.split(',')				# Delineate message values
					data = (v[3].split('*'))[0]		# Strip XOR value off data
					data = int(data, 16)			# Cast data as int
					bnd = bin(data)				# Convert to binary string
					cmd = bnd[2:5]					# Format "0bXXXXXXXXXXXXX" strip "ob"
					print('bnd: ' + bnd + ' cmd: ' + cmd)
					val = bnd[5:]					
					ch_raw = int(val, 2) + 1000		# Convert rest of value to string
					# Pack appropriate MAVLINK message, send along.
					if cmd == '001':	# RC Override Channel 1
				#		mav_pack = self.mav.MAVLink_rc_channels_override_message(
						print('RC Override 001 received');															100, 100, ch_raw, ret, 
				#															ret, ret, ret, ret, ret, ret)
				#		self.packetQueue.put(mav_pack.pack(mav))
					elif cmd == '010':	# RC Override Channel 2
						print('RC Override 010 received')
				#		mav_pack = self.mav.MAVLink_rc_channels_override_message(
				#															100, 100, ret, ch_raw, 
				#															ret, ret, ret, ret, ret, ret)
				#		self.packetQueue.put(mav_pack.pack(mav))
				#	elif cmd == '010':	# RC Override Channel 3
				#		mav_pack = self.mav.MAVLink_rc_channels_override_message(
				#															100, 100, ret, ret, 
				#															ch_raw, ret, ret, ret, ret, ret)
				#		self.packetQueue.put(mav_pack.pack(mav))
				#	elif cmd == '011':	# RC Override Channel 
				#		mav_pack = self.mav.MAVLink_rc_channels_override_message(
				#															100, 100, ret, ret, 
				#															ret, ch_raw, ret, ret, ret, ret)
				#		self.packetQueue.put(mav_pack.pack(mav))
				#	elif cmd == '100':
				#		pass
				#	elif cmd == '101':
				#		pass
				#	elif cmd == '110':
				#		pass
				#	elif cmd == '111':
				#		pass
	def receive(self):
		msg = RxQueue.get()
		return msg
