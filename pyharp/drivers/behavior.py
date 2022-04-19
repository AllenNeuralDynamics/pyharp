"""Behavior Device Driver."""

from pyharp.messages import HarpMessage, ReplyHarpMessage
from pyharp.device_names import device_names
from pyharp.device import Device
import serial
from serial.serialutil import SerialException


# Type, Base Address, "Description."
REGISTERS = \
{   # RJ45 "PORT" (0, 1, 2)  Digital Inputs
    "PORT_DIS" : ("U8", 32, "Reflects the state of DI digital lines of each Port."),

    # Manipulate any of the boards digital outputs.
    "OUTPUTS_SET": ("U16", 34, "Set the corresponding output."),
    "OUTPUTS_CLR": ("U16", 35, "Clear the corresponding output."),
    "OUTPUTS_TOGGLE": ("U16", 36, "Toggle the corresponding output."),
    "OUTPUTS_OUT": ("U16", 37, "Control corresponding output."),

    # RJ45 "PORT" (0, 1, 2) Digital IOs
    "PORT_DIOS_SET": ("U8", 38, "Set the corresponding DIO."),
    "PORT_DIOS_CLEAR": ("U8", 39, "Clear the corresponding DIO."),
    "PORT_DIOS_TOGGLE": ("U8", 40, "Toggle the corresponding DIO."),
    "PORT_DIOS_OUT": ("U8", 41, "Control the corresponding DIO."),
    "PORT_DIOS_CONF": ("U8", 42, "Set the DIOs direction (1 is output)."),
    "PORT_DIOS_IN": ("U8", 43, "State of the DIOs."),
}



class Behavior:
    """Driver for BehaviorDevice."""

    # On Linux, the symlink to the first detected harp device.
    # Name set in udev rules and will increment with subsequent devices.
    DEVICE_NAME = "Behavior"
    DEFAULT_PORT_NAME = "/dev/harp_device_00"
    ID = 1216

    # TODO: put this in a base class?
    READ_MSG_LOOKUP = \
    {
        "U8": HarpMessage.ReadU8,
        "U16": HarpMessage.ReadU16,
        "S16": HarpMessage.ReadS16,
    }

    WRITE_MSG_LOOKUP = \
    {
        "U8": HarpMessage.WriteU8,
        "U16": HarpMessage.WriteU16,
        "S16": HarpMessage.WriteS16,
    }


    def __init__(self, port_name=None, output_filename=None):
        """Class constructor. Connect to a device."""

        self.device = None

        try:
            if port_name is None:
                self.device = Device(self.__class__.DEFAULT_PORT_NAME, output_filename)
            else:
                self.device = Device(port_name, output_filename)
        except (FileNotFoundError, SerialException):
            print("Error: Failed to connect to Behavior Device. Is it plugged in?")
            raise

        if self.device.WHO_AM_I != self.__class__.ID:
            raise IOError("Error: Did not connect to Harp Behavior Device.")


    # TODO: put this in a base class?
    def get_reg_info(self, reg_name: str) -> str:
        """get info for this device's particular reg."""
        try:
            return REGISTERS[reg_name][2]
        except KeyError:
            raise KeyError(f"reg: {reg_name} is not a register in "
                            "{self.__class__.name} Device's register map.")


# Board inputs, outputs, and some settings configured as @properties.
    # INPUTS
    @property
    def all_port_input_states(self):
        """return the state of all PORT digital inputs."""
        reg_type, reg_index, _ = REGISTERS["PORT_DIS"]
        read_message_type = self.__class__.READ_MSG_LOOKUP[reg_type]
        return self.device.send(read_message_type(reg_index).frame).payload_as_int()

    @property
    def port0_i0(self):
        """return the state of port0 digital in 0."""
        return self.all_port_input_states & 0x01

    @property
    def port1_i0(self):
        """return the state of port0 digital in 0."""
        return (self.all_port_input_states >> 1) & 0x01

    @property
    def port2_i0(self):
        """return the state of port2 digital in 0."""
        return (self.all_port_input_states >> 2) & 0x01

    # IOs
    def set_port_io_states(self, bitmask : int):
        """set the state of all PORT digital ios. (1 is output.)"""
        reg_type, reg_index, _ = REGISTERS["PORT_DIOS_CONF"]
        write_message_type = self.__class__.WRITE_MSG_LOOKUP[reg_type]
        self.device.send(write_message_type(reg_index, bitmask).frame)

    @property #FIXME: this doesn't seem to work
    def all_port_io_states(self):
        """return the state of all PORT digital ios."""
        reg_type, reg_index, _ = REGISTERS["PORT_DIOS_IN"]
        read_message_type = self.__class__.READ_MSG_LOOKUP[reg_type]
        return self.device.send(read_message_type(reg_index).frame).payload_as_int()

    @property
    def port0_io0(self):
        """read the digital io state."""
        return self.all_port_io_states & 0x01

    @port0_io0.setter
    def port0_io0(self, value: int):
        """write port0 digital io state."""
        pass

    @property
    def port1_io0(self):
        """read the digital io state."""
        return (self.all_port_io_states >> 1) & 0x01

    @port0_io0.setter
    def port1_io0(self, value: int):
        """write port0 digital io state."""
        self.set_outputs(value&0x01)

    @property
    def port2_io0(self):
        """read the digital io state."""
        return (self.all_port_io_states >> 2) & 0x01

    @port0_io0.setter
    def port2_io0(self, value: int):
        """write port0 digital io state."""
        pass


    # OUTPUTS
    @property
    def all_output_states(self):
        """return the state of all PORT digital inputs."""
        reg_type, reg_index, _ = REGISTERS["OUTPUTS_OUT"]
        read_message_type = self.__class__.READ_MSG_LOOKUP[reg_type]
        return self.device.send(read_message_type(reg_index).frame).payload_as_int()

    @all_output_states.setter
    def all_output_states(self, bitmask : int):
        """set the state of all PORT digital inputs."""
        reg_type, reg_index, _ = REGISTERS["OUTPUTS_OUT"]
        write_message_type = self.__class__.WRITE_MSG_LOOKUP[reg_type]
        return self.device.send(write_message_type(reg_index, bitmask).frame)

    def set_outputs(self, bitmask : int):
        """set digital outputs to logic 1 according to bitmask."""
        reg_type, reg_index, _ = REGISTERS["OUTPUTS_SET"]
        write_message_type = self.__class__.WRITE_MSG_LOOKUP[reg_type]
        return self.device.send(write_message_type(reg_index, bitmask).frame)

    def clear_outputs(self, bitmask : int):
        """clear digital outputs (specified with logic 1) according to bitmask."""
        reg_type, reg_index, _ = REGISTERS["OUTPUTS_CLR"]
        write_message_type = self.__class__.WRITE_MSG_LOOKUP[reg_type]
        return self.device.send(write_message_type(reg_index, bitmask).frame)

    @property
    def D0(self):
        """read the digital output D0 state."""
        return (self.all_output_states >> 10) & 0x01

    @D0.setter
    def D0(self, value):
        """set the digital output D0 state."""
        if value:
            self.set_outputs(1 << 10)
        else:
            self.clear_outputs(1 << 10)

    @property
    def D1(self):
        """read the digital output D1 state."""
        return (self.all_output_states >> 11) & 0x01

    @D1.setter
    def D1(self, value):
        """set the digital output D1 state."""
        if value:
            self.set_outputs(1 << 11)
        else:
            self.clear_outputs(1 << 11)

    @property
    def D2(self):
        """read the digital output D2 state."""
        return (self.all_output_states >> 12) & 0x01

    @D2.setter
    def D2(self, value):
        """set the digital output D2 state."""
        if value:
            self.set_outputs(1 << 12)
        else:
            self.clear_outputs(1 << 12)

    @property
    def D3(self):
        """read the digital output D3 state."""
        return (self.all_output_states >> 10) & 0x01

    @D3.setter
    def D3(self, value):
        """set the digital output D3 state."""
        if value:
            self.set_outputs(1 << 13)
        else:
            self.clear_outputs(1 << 13)

    @property
    def port0_D0(self):
        return self.all_output_states & 0x01

    @port0_D0.setter
    def port0_D0(self, value):
        if value:
            self.set_outputs(1)
        else:
            self.clear_outputs(1)

    @property
    def port1_D0(self):
        return (self.all_output_states >> 1) & 0x01

    @port1_D0.setter
    def port1_D0(self, value):
        if value:
            self.set_outputs(1 << 1)
        else:
            self.clear_outputs(1 << 1)

    @property
    def port2_D0(self):
        return (self.all_output_states >> 2) & 0x01

    @port2_D0.setter
    def port2_D0(self, value):
        if value:
            self.set_outputs(1 << 2)
        else:
            self.clear_outputs(1 << 2)


    @property
    def port0_12V(self):
        return (self.all_output_states >> 3) & 0x01

    @port0_12V.setter
    def port0_12V(self, value):
        if value:
            self.set_outputs(1 << 3)
        else:
            self.clear_outputs(1 << 3)

    @property
    def port1_12V(self):
        return (self.all_output_states >> 4) & 0x01

    @port1_12V.setter
    def port1_12V(self, value):
        if value:
            self.set_outputs(1 << 4)
        else:
            self.clear_outputs(1 << 4)

    @property
    def port2_12V(self):
        return (self.all_output_states >> 5) & 0x01

    @port2_12V.setter
    def port2_12V(self, value):
        if value:
            self.set_outputs(1 << 5)
        else:
            self.clear_outputs(1 << 5)


    def __enter__(self):
        """Setup for the 'with' statement"""
        return self


    def __exit__(self, *args):
        """Cleanup for the 'with' statement"""
        self.device.disconnect()


    def __del__(self):
        """Cleanup when Device gets garbage collected."""
        self.device.disconnect()
