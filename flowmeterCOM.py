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

def crc16_modbus(data: bytearray) -> int:
    """
    Calculate CRC16 Modbus checksum for the given byte array.
    
    :param data: Input data as a bytearray
    :return: CRC16 Modbus checksum as an integer
    """
    crc = 0xFFFF  # Initialize CRC to 0xFFFF
    for byte in data:
        crc ^= byte  # XOR byte into least significant byte of CRC
        for _ in range(8):  # Process each bit
            if crc & 0x0001:  # If the least significant bit is set
                crc = (crc >> 1) ^ 0xA001  # Right shift and XOR with polynomial
            else:
                crc >>= 1  # Just shift right
    return crc

def append_crc(data: bytearray) -> bytearray:
    """
    Append CRC16 Modbus checksum as two bytes to the end of a byte array.
    
    :param data: Input data as a bytearray
    :return: New bytearray with CRC appended
    """
    crc = crc16_modbus(data)
    lsb = crc & 0xFF  # Extract least significant byte
    msb = (crc >> 8) & 0xFF  # Extract most significant byte
    data_with_crc = data + bytearray([lsb, msb])  # Append LSB and MSB to the data
    return data_with_crc


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
        port='com38',
        baudrate=115200,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout = 1
    )

    #ser.open()


    tmp = bytearray([0x01,0x04,0x01,0x41,0x00,0x02])
    tmp_crc = append_crc(tmp)
    print('TX:', ' '.join([hex(i) for i in tmp_crc]))
    while True:
          ser.write(tmp_crc)
          line = ser.readline()
          print('RX:', ' '.join([hex(i) for i in line]))
          time.sleep(5)  


    ser.close()             # close port
