# Python Specific Libraries
import serial
import numpy as np
from ctypes import *
import os
import sys
import time

# Project Specific Libraries
'''
currentdir = os.getcwd()
parentdir  = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
'''

class Flag_bits(BigEndianStructure):
	_fields_ = [
	("logout",c_ubyte,1),
	("userswitch",c_ubyte,2),
	("suspend",c_ubyte,3),
	("idle",c_ubyte,1),
	("reserved",c_ubyte,1),
	("array",c_ubyte*5)
	]
	
class Flags(Union):
	_fields_ = [
	("b",Flag_bits),
	("arrayfull",c_ubyte*6)
	]
	
if __name__ == '__main__':
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port='com39',
        baudrate=115200,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout = 1
    )

    #ser.open()


    tmp = bytearray([0x01,0x04,0x01,0x41,0x00,0x02,0x20,0x23])
 
    while True:
          ser.write(tmp)
          line = ser.readline()
          print('RX:', ' '.join([hex(i) for i in line]))
          time.sleep(5)  


    ser.close()             # close port
