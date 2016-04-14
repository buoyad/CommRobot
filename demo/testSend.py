#! /usr/bin/env python
import serial, time, sys

whoi = serial.Serial(timeout=None)
whoi.port = '/dev/ttyUSB0'
whoi.baudrate = 19200

whoi.open();
whoi.flushInput()
whoi.flushOutput()
                                                                
whoi.write('$CCCFG,REV,0\r\n');                                                        
whoi.write('$CCCFG,ASD,0\r\n');                                                        
whoi.write('$CCCFG,SRC,1\r\n');                                                        
whoi.write('$CCCFG,RXA,0\r\n');                                                        
whoi.write('$CCCFG,RXD,1\r\n');                                                        
whoi.write('$CCCFG,DTO,5\r\n');                                                        
whoi.write('$CCCFG,AGN,0\r\n');                                                        
whoi.write('$CCCFG,XST,0\r\n'); 

time.sleep(1);

whoi.write('$CCMUC,0,1,1FFF'+'\r\n')

x=0
while True:                                                                     
	print(whoi.readline())
	time.sleep(.05)
	x=x+1
	if (x >= 200): 
		break
	   
                                                                             
ser.close();  
