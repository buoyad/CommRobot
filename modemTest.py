#! /usr/bin/env python
import serial

m = serial.Serial()
m.port = '/dev/ttyUSB2'
m.baudrate = 19200

m.open()

m.flushInput()

while True:
	print m.readline()
