#!/usr/bin/env python3
from pyharp.device import Device, DeviceMode
from pyharp.messages import WriteU8ArrayMessage
from pyharp.messages import MessageType
from app_registers import AppRegs
from struct import pack, unpack
import os
import serial.tools.list_ports

# Open serial connection with the first Valve Controller.
com_port = None
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    if desc.startswith("valve-controller"):
        print("{}: {} [{}]".format(port, desc, hwid))
        com_port = port
        break
device = Device(com_port)
device.info()                           # Display device's info on screen

valve_config = \
(
    0.8, # Hit output [0-1.0]
    0.25, # Hold output [0-1.0]
    500000,   # Hit duration in microseconds [0-65535]
)
data_fmt = "<ffL"
print("Configuring Valve Output 0.")
reply = device.send(WriteU8ArrayMessage(
            AppRegs.ValveConfigs0, data_fmt, valve_config).frame)
print("reply:")
print(unpack(data_fmt, bytes(reply.payload)))

# Close connection
device.disconnect()
