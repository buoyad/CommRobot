#! /usr/bin/env python
import serial

s = serial.Serial()
s.port = '/dev/ttyUSB0'
s.baudrate = 19200
s.timeout = .5
s.open()

while True:
	print s.readline()
