from typing import Union
from functools import partial
import logging
import queue
import threading
import serial
import serial.threaded

from pyharp.messages import HarpMessage, MessageType


class HarpSerialProtocol(serial.threaded.Protocol):
    _read_q: queue.Queue

    def __init__(self, _read_q: queue.Queue, *args, **kwargs):
        self._read_q = _read_q
        super().__init__(*args, **kwargs)

    def connection_made(self, transport: serial.threaded.ReaderThread) -> None:
        print(f"Connected to {transport.serial.port}")
        return super().connection_made(transport)

    def data_received(self, data: bytes) -> None:
        for byte in data:
            self._read_q.put(byte)
        return super().data_received(data)

    def connection_lost(self, exc: Union[BaseException, None]) -> None:
        print(f"Lost connection!")
        return super().connection_lost(exc)


class HarpSerial:

    msg_q: queue.Queue
    event_q: queue.Queue

    def __init__(self, serial_port: str, **kwargs):
        self._ser = serial.Serial(serial_port, **kwargs)

        self.log = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        self._read_q = queue.Queue()
        self.msg_q = queue.Queue()
        self.event_q = queue.Queue()

        self._reader = serial.threaded.ReaderThread(
            self._ser,
            partial(HarpSerialProtocol, self._read_q),
        )
        self._reader.start()
        transport, protocol = self._reader.connect()

        self._parse_thread = threading.Thread(
            target=self.parse_harp_msgs_threaded,
            daemon=True,
        )
        self._parse_thread.start()

    def close(self):
        self._reader.close()

    def write(self, data):
        self._reader.write(data)

    def parse_harp_msgs_threaded(self):
        while True:
            message_type = self._read_q.get(1)  # byte array with only one byte
            message_length = self._read_q.get(1)
            message_content = bytes([self._read_q.get() for _ in range(message_length)])
            self.log.debug(f"reply (type): {message_type}")
            self.log.debug(f"reply (length): {message_length}")
            self.log.debug(f"reply (payload): {message_content}")

            frame = bytearray()
            frame.append(message_type)
            frame.append(message_length)
            frame += message_content
            msg = HarpMessage.parse(frame)

            if msg.message_type == MessageType.EVENT:
                self.event_q.put(msg)
            else:
                self.msg_q.put(msg)
