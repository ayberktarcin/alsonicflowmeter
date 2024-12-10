# Python Specific Libraries
import serial
import numpy as np
from ctypes import *
import os
import sys
import time
import struct
from tabulate import tabulate

# Project Specific Libraries
'''
currentdir = os.getcwd()
parentdir  = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
'''

# Constants
SERIAL_PORT = 'com38'  # Replace with your serial port
BAUD_RATE = 115200
DEV_ID = 1
READ_COMMAND = 4
PERIODIC_TIME_SECOND = 2

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

# Function to parse the received data
def parse_data(line):
    """
    Parse the received data from the device.
    
    :param line: Byte array containing the received data
    :return: Parsed dictionary with field values
    """
    if len(line) < 9:  # Minimum length: DevID + Command + Bytes + Word1 + CRC16
        raise ValueError("Data length is too short")

    # Extract fields
    dev_id = line[0]
    command = line[1]
    num_data_bytes = line[2]
    data = line[3:-2]  # Exclude CRC
    crc = struct.unpack("<H", line[-2:])[0]  # Little-endian CRC16

    # Parse words (assuming little-endian and appropriate data size)
    fields = {}
    offset = 0
    register_mapping = {
        0: ("Flow", "Int32", 0.1, "l/h"),
        2: ("Temp Internal", "Int16", 0.01, "°C"),
        3: ("Temp Remote", "Int16", 0.01, "°C "),
        4: ("Temp Difference", "Int16", 0.01, "°C "),
        5: ("Pressure", "Int16", 0.001, "Bar "),
        6: ("Power", "Int32", 0.01, "kW "),
    }

    while offset < len(data):
        register = offset // 2  # Registers are 2 bytes each
        if register in register_mapping:
            name, dtype, resolution, unit = register_mapping[register]
            if dtype == "Int32":
                value = struct.unpack_from(">i", data, offset)[0] # Big-endian 32-bit integer
                offset += 4
            elif dtype == "Int16":
                value = struct.unpack_from(">h", data, offset)[0] # Big-endian 16-bit integer
                offset += 2
            else:
                raise ValueError(f"Unknown data type {dtype}")

            # Apply resolution
            value *= resolution
            formatted_value = f"{value:.6g}"  # Use general format with max 6 significant digits
            fields[name] = f"{formatted_value} {unit}"
        else:
            offset += 2  # Skip unrecognized register

    return {
        "Device ID": dev_id,
        "Command": command,
        "Data Fields": fields,
        "CRC": f"0x{crc:04X}",  # Show CRC as hex with leading 0x
    }

# Function to parse the received data
def parse_configdata(line):
    """
    Parse the received data from the device.
    
    :param line: Byte array containing the received data
    :return: Parsed dictionary with field values
    """
    if len(line) < 9:  # Minimum length: DevID + Command + Bytes + Word1 + CRC16
        raise ValueError("Data length is too short")

    # Extract fields
    dev_id = line[0]
    command = line[1]
    num_data_bytes = line[2]
    data = line[3:-2]  # Exclude CRC
    crc = struct.unpack("<H", line[-2:])[0]  # Little-endian CRC16

    # Parse words (assuming little-endian and appropriate data size)
    fields = {}
    offset = 0
    register_mapping = {
        0: ("Device ID", "Int16", 1.0, " "),
        1: ("BAUD Rate", "Int32", 1.0, "bps"),
        3: ("Reserved", "Int16", 0.0, " "),
        4: ("Parity", "Int16", 1.0, " "),
        5: ("Stop bit", "Int16", 1.0, " "),
    }

    while offset < len(data):
        register = offset // 2  # Registers are 2 bytes each
        if register in register_mapping:
            name, dtype, resolution, unit = register_mapping[register]
            if dtype == "Int32":
                value = struct.unpack_from(">i", data, offset)[0] # Big-endian 32-bit integer
                offset += 4
            elif dtype == "Int16":
                value = struct.unpack_from(">h", data, offset)[0] # Big-endian 16-bit integer
                offset += 2
            else:
                raise ValueError(f"Unknown data type {dtype}")

            # Apply resolution
            value *= resolution
            fields[name] = f"{value} {unit}"
        else:
            offset += 2  # Skip unrecognized register

    return {
        "Device ID": dev_id,
        "Command": command,
        "Data Fields": fields,
        "CRC": f"0x{crc:04X}",  # Show CRC as hex with leading 0x
    }

# Main communication loop
def periodic_read():
    tmp = bytearray([DEV_ID,READ_COMMAND,0x00,0x00,0x00,0x08])
    tmp_crc = append_crc(tmp)
    #print('TX:', ' '.join([hex(i) for i in tmp_crc]))
    ser.write(tmp_crc)  # Write the command to the device
    line = ser.readline()  # Read the response
    #print("RX:", " ".join([hex(i) for i in line]))
    try:
        parsed = parse_data(line)  # Parse the received data
        data_fields = parsed.get("Data Fields", {})

        # Prepare data for tabulate
        headers = list(data_fields.keys())
        values = list(data_fields.values())

        # Print as a single row
        print(tabulate([values], headers=headers, tablefmt="grid"))
    except Exception as e:
        print("Error:", e)

def config_read():
    tmp = bytearray([DEV_ID,READ_COMMAND,0x01,0x40,0x00,0x06])
    tmp_crc = append_crc(tmp)
    #print('TX:', ' '.join([hex(i) for i in tmp_crc]))
    ser.write(tmp_crc)  # Write the command to the device
    line = ser.readline()  # Read the response
    #print("RX:", " ".join([hex(i) for i in line]))
    try:
        parsed = parse_configdata(line)
        print("New Data:")
        for key, value in parsed.items():
            if isinstance(value, dict):
                for sub_key, sub_value in value.items():
                    print(f"  {sub_key}: {sub_value}")
            else:
                print(f"{key}: {value}")
    except Exception as e:
        print("Error:", e)

if __name__ == '__main__':
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout = 1
    )
    config_read()
    #ser.open()
    while True:
        periodic_read()
        time.sleep(PERIODIC_TIME_SECOND)  


    ser.close()             # close port
