#!/usr/bin/env python3
from pyharp.device import Device, DeviceMode
from pyharp.messages import HarpMessage
from pyharp.messages import MessageType
from app_registers import AppRegs
from struct import pack, unpack
from time import sleep
import os
import serial.tools.list_ports

import logging
logging.basicConfig(level=logging.DEBUG)
#logging.getLogger().addHandler(logging.StreamHandler())


#logger = logging.getLogger()
#logger.setLevel(logging.DEBUG)

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

print("Enabling Valve0")
valves = 0x0001 # valve0
reply = device.send(HarpMessage.WriteU16(AppRegs.ValvesSet, valves).frame)
print("reply:")
print(reply)
print()
sleep(1.0)
print("Disabling Valve0")
reply = device.send(HarpMessage.WriteU16(AppRegs.ValvesClear, valves).frame)
print("reply:")
print(reply)
print()

# Close connection
device.disconnect()
