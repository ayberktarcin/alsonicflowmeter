# Modbus Communication with Alsonic Flowmeter  

## Overview  
This Python script facilitates Modbus communication with an Alsonic Flowmeter connected to a PC. The script is designed to send and receive Modbus messages for querying the flowmeter's parameters and sensor data.  

## Requirements  
To set up and run this project, you will need the following:  
1. **Hardware Requirements**:  
   - A PC with a serial communication port (or USB-to-RS485 converter, if needed).  
   - An Alsonic Flowmeter.  
   - A Modbus communication setup for the flowmeter and PC.  

2. **Software Requirements**:  
   - Python 3.7 or later.  
   - Required Python libraries (see the [Installation](#installation) section).  

## Installation  
1. Clone this repository or download the Python script.  
2. Install the required Python libraries using pip:  
   ```bash  
   pip install pyserial  
