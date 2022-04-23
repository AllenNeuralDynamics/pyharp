import serial
import time
from threading import Thread
from typing import Optional, Union
from pathlib import Path

from pyharp.messages import HarpMessage, ReplyHarpMessage
from pyharp.messages import CommonRegisters
from pyharp.device_names import device_names
from enum import Enum


class DeviceMode(Enum):
    Standby = 0
    Active = 1
    Reserved = 2
    Speed = 3


class Device:
    """
    https://github.com/harp-tech/protocol/blob/master/Device%201.1%201.0%2020220402.pdf
    """

    _ser: serial.Serial
    _reply_cache: dict
    _keep_thread_active: bool
    _dump_file_path: Path

    WHO_AM_I: int
    WHO_AM_I_DEVICE: str
    HW_VERSION_H: int
    HW_VERSION_L: int
    ASSEMBLY_VERSION: int
    HARP_VERSION_H: int
    HARP_VERSION_L: int
    FIRMWARE_VERSION_H: int
    FIRMWARE_VERSION_L: int
    DEVICE_NAME: str

    def __init__(self, serial_port: str, dump_file_path: Optional[str] = None):
        self._reply_cache = {}
        self._keep_thread_active = False
        self._serial_port = serial_port
        if dump_file_path is None:
            self._dump_file_path = None
        else:
            self._dump_file_path = Path() / dump_file_path
        self.connect()
        self.load()

    def load(self) -> None:
        self.WHO_AM_I = self.read_who_am_i()
        self.WHO_AM_I_DEVICE = self.read_who_am_i_device()
        self.HW_VERSION_H = self.read_hw_version_h()
        self.HW_VERSION_L = self.read_hw_version_l()
        self.ASSEMBLY_VERSION = self.read_assembly_version()
        self.HARP_VERSION_H = self.read_harp_h_version()
        self.HARP_VERSION_L = self.read_harp_l_version()
        self.FIRMWARE_VERSION_H = self.read_fw_h_version()
        self.FIRMWARE_VERSION_L = self.read_fw_l_version()
        self.DEVICE_NAME = self.read_device_name()

    def info(self) -> None:
        print("Device info:")
        print(f"* Who am I: ({self.WHO_AM_I}) {self.WHO_AM_I_DEVICE}")
        print(f"* HW version: {self.HW_VERSION_H}.{self.HW_VERSION_L}")
        print(f"* Assembly version: {self.ASSEMBLY_VERSION}")
        print(f"* HARP version: {self.HARP_VERSION_H}.{self.HARP_VERSION_L}")
        print(f"* Firmware version: {self.FIRMWARE_VERSION_H}.{self.FIRMWARE_VERSION_L}")
        print(f"* Device user name: {self.DEVICE_NAME}")
        print(f"* Mode: {self.read_device_mode().name}")

    def read(self):
        pass

    def connect(self) -> None:
        self._ser = serial.Serial(
            self._serial_port,  # "/dev/tty.usbserial-A106C8O9"
            baudrate=1000000,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            rtscts=True,
        )
        # TODO: put the device in standby mode.
        # Flush the buffer to get rid of incoming messages.


    def launch(self):
        self._keep_thread_active = True
        self._thread_worker = Thread(target=self.spin_worker, daemon=True).start()


    def disconnect(self) -> None:
        self._ser.close()

    def read_who_am_i(self) -> int:
        address = CommonRegisters.WHO_AM_I

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU16(address), dump=False
        )

        return reply.payload_as_int()

    def read_who_am_i_device(self) -> str:
        address = CommonRegisters.WHO_AM_I

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU16(address), dump=False
        )

        return device_names.get(reply.payload_as_int())

    def read_hw_version_h(self) -> int:
        address = CommonRegisters.HW_VERSION_H

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_hw_version_l(self) -> int:
        address = CommonRegisters.HW_VERSION_L

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_assembly_version(self) -> int:
        address = CommonRegisters.ASSEMBLY_VERSION

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_harp_h_version(self) -> int:
        address = CommonRegisters.HARP_VERSION_H

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_harp_l_version(self) -> int:
        address = CommonRegisters.HARP_VERSION_L

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_fw_h_version(self) -> int:
        address = CommonRegisters.FIRMWARE_VERSION_H

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_int()

    def read_fw_l_version(self) -> int:
        address = CommonRegisters.FIRMWARE_VERSION_L

        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
)

        return reply.payload_as_int()

    def read_device_name(self) -> str:
        address = CommonRegisters.DEVICE_NAME

        # reply: Optional[bytes] = self.send(HarpMessage.ReadU8(address), 13 + 24)
        reply: ReplyHarpMessage = self.send(
            HarpMessage.ReadU8(address), dump=False
        )

        return reply.payload_as_string()

    def read_device_mode(self) -> DeviceMode:
        address = CommonRegisters.OPERATION_CTRL
        reply = self.send(HarpMessage.ReadU8(address))
        print(reply)
        return DeviceMode(reply.payload_as_int() & 0x03)

# TODO: Not sure if we want to implement these. Delete if no.
    def set_mode(self, mode: DeviceMode) -> ReplyHarpMessage:
        """Change the device's OPMODE. Reply can be ignored."""
        address = CommonRegisters.OPERATION_CTRL
        # Read register first.
        reg_value = self.send(HarpMessage.ReadU8(address)).payload_as_int()
        reg_value &= ~0x03 # mask off old mode.
        reg_value |= mode.value
        reply = self.send(HarpMessage.WriteU8(address, reg_value))
        return reply

    def enable_status_led(self):
        """enable the device's status led if one exists."""
        address = CommonRegisters.OPERATION_CTRL
        # Read register first.
        reg_value = self.send(HarpMessage.ReadU8(address)).payload_as_int()
        reg_value |= (1 << 6)
        reply = self.send(HarpMessage.WriteU8(address, reg_value))

    def enable_status_led(self):
        """enable the device's status led if one exists."""
        address = CommonRegisters.OPERATION_CTRL
        # Read register first.
        reg_value = self.send(HarpMessage.ReadU8(address)).payload_as_int()
        reg_value &= ~(1 << 6)
        reply = self.send(HarpMessage.WriteU8(address, reg_value))

    def enable_alive_en(self):
        """Enable ALIVE_EN such that the device sends an event each second."""
        address = CommonRegisters.OPERATION_CTRL
        # Read register first.
        reg_value = self.send(HarpMessage.ReadU8(address)).payload_as_int()
        reg_value |= (1 << 7)
        reply = self.send(HarpMessage.WriteU8(address, reg_value))

    def disable_alive_en(self):
        """disable ALIVE_EN such that the device does not send an event each second."""
        address = CommonRegisters.OPERATION_CTRL
        # Read register first.
        reg_value = self.send(HarpMessage.ReadU8(address)).payload[0]
        reg_value &= ((1<< 7) ^ 0xFF) # bitwise ~ operator substitute for Python ints.
        reply = self.send(HarpMessage.WriteU8(address, reg_value))


    def send(self, message: HarpMessage, dump: bool = True) -> ReplyHarpMessage:
        """Send a harp message; return the device's reply."""
        self._ser.write(message.frame)
        # clear the cache with a timestamp of when we made the read request.
        # FIXME: do we want to timestamp after the data goes out with flush?
        self._reply_cache[message.address] = (time.perf_counter(), None)

        reply: ReplyHarpMessage = self.read(message.address)
        if dump:
            self._dump_reply(reply.frame)
        return reply


    def spin_worker(self) -> None:
        """ called in a thread/process."""

        # Bring in replies and cached them.
        while True and self._keep_thread_active:
            reply = self._read()
            # Update the cache.
            # TODO: timestamp incoming data with local time?
            self._reply_cache[reply.address] = (time.perf_counter(), reply)


    def read(self, address) -> ReplyHarpMessage:
        """Read from cache if unread data has been cached. Blocking read if not."""
        # Blocking read if we're not threading.
        if not self._keep_thread_active:
            return self._read()

        # Block until a reply gets cached.
        # Discard time checking for now.
        # Separate thread will update the cache periodically.
        _, reply = self._reply_cache.get(address, (None, None))
        while reply is None:
            _, reply = self._reply_cache.get(address, (None, None))
        # remove the data once read.
        del self._reply_cache[address]
        return reply


    def _read(self) -> Union[ReplyHarpMessage, None]:
        """(Blocking) Read an incoming message from the serial port."""
        self._ser.flush() # wait for outgoing data to leave serial port.

        # block until we get at least one byte.
        while True:
            if self._ser.inWaiting():
                break
        try:
            message_type = self._ser.read(1)[0]  # byte array with only one byte
            message_length = self._ser.read(1)[0]
            message_content = self._ser.read(message_length)

            frame = bytearray()
            frame.append(message_type)
            frame.append(message_length)
            frame += message_content

            return HarpMessage.parse(frame)
        except IndexError:
            return None


    def _dump_reply(self, reply: bytes):
        assert self._dump_file_path is not None
        with self._dump_file_path.open(mode="ab") as f:
            f.write(reply)


    def __enter__(self):
        return self


    def __exit__(self):
        self.__del__()


    def __del__(self):
        if self._keep_thread_active:
            self._keep_thread_active = False
            self._thread_worker.join()
            while self._thread_worker.isAlive():
                pass
        self.disconnect()
