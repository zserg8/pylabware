"""PyLabware driver for Mettler Toledo MS3002S balance."""

from typing import Optional, Union
import serial

# Core imports
from .. import parsers as parser
from ..controllers import LabDevice, in_simulation_device_returns
from ..exceptions import PLConnectionError, PLDeviceCommandError
from ..models import LabDeviceCommands, ConnectionParameters


class MS3002SBalanceCommands(LabDeviceCommands):
    """Collection of command definitions for Mettler Toledo MS3002S balances.
    """

    # ##########################  Constants ##################################
    # Default reply to GET_TYPE command
    DEFAULT_TYPE = "MS3002S/01"
    # Maximum allowed weight, in grams
    MAX_WEIGHT = 3200

    # ################### Control commands ###################################
    # Get list of commands
    GET_CMD_LIST = {"name": "I0", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get command interface level and version
    GET_CMD_VER = {"name": "I1", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get balance info string
    GET_INFO = {"name": "I2", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get software version
    GET_SW_VER = {"name": "I3", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get serial number
    GET_SER_NUM = {"name": "I4", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get software id number
    GET_SW_ID = {"name": "I5", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get individual unit id string
    GET_ID = {"name": "I10", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get balance type/model
    GET_NAME = {"name": "I11", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Set tare weight
    SET_TARE = {"name": "T", "reply":{"type": str}}
    # Set tare weight immediately
    SET_TARE_IMMEDIATE = {"name": "TI", "reply":{"type": str}}
    # Get tare weight/ set tare weight to a specified value
    GET_TARE = {"name": "TA", "reply":{"type": float, "parser": parser.splitter, "args":[" ", -2, -1]}}
    # Clear tare weight
    CLEAR_TARE = {"name": "TAC", "reply":{"type": str}}
    # Zero weight
    ZERO = {"name": "Z", "reply":{"type": str}}
    # Zero weight immediately
    ZERO_IMMEDIATE = {"name": "ZI", "reply":{"type": str}}
    # Get stable weight
    GET_WEIGHT = {"name": "S", "reply":{"type": float, "parser": parser.splitter, "args":[" ", -2, -1]}}
    # Get weight immediately
    GET_WEIGHT_IMMEDIATE = {"name": "SI", "reply":{"type": float, "parser": parser.splitter, "args":[" ", -2, -1]}}
    # Get stable weight with unit
    GET_WEIGHT_UNIT = {"name": "SU", "reply":{"type": str, "parser": parser.splitter, "args":[" ", -2, -1]}}
    # Get weight immediately
    GET_WEIGHT_UNIT_IMMEDIATE = {"name": "SIU", "reply":{"type": str, "parser": parser.splitter, "args":[" ", -2, -1]}}

    # ################### Configuration commands #############################

    # Reset device
    RESET = {"name": "@", "reply": {"type": str}}


class MS3002SBalance(LabDevice):
    """
    This provides a Python class for the Mettler Toledo MS3002S balance
    based on the original operation manual 11781259 0309/2.12.
    """

    def __init__(self, device_name: str, connection_mode: str, address: Optional[str], port: Union[str, int]):
        """Default constructor
        """

        # Load commands from helper class
        self.cmd = MS3002SBalanceCommands

        # Connection settings
        connection_parameters: ConnectionParameters = {}
        connection_parameters["port"] = port
        connection_parameters["address"] = address
        connection_parameters["baudrate"] = 9600
        connection_parameters["bytesize"] = serial.EIGHTBITS
        connection_parameters["parity"] = serial.PARITY_NONE

        super().__init__(device_name, connection_mode, connection_parameters)

        # Protocol settings
        self.command_terminator = "\r\n"
        self.reply_terminator = "\r\n"
        self.args_delimiter = " "

    def initialize_device(self):
        """Issue reset command.
        """

        self.send(self.cmd.RESET)
        self.logger.info("Device initialized.")

    @in_simulation_device_returns([MS3002SBalanceCommands.DEFAULT_TYPE])
    def is_connected(self) -> bool:
        """ Check if the device is connected via GET_NAME command.
        """

        try:
            reply = self.send(self.cmd.GET_NAME)
        except PLConnectionError:
            return False
        reply = reply[-1]
        return self.cmd.DEFAULT_TYPE in reply

    # TODO Maybe put power on/off in start/stop ?
    def start(self):
        """Not supported on this device.
        """

    def stop(self) -> bool:
        """Not supported on this device.
        """

    def is_idle(self) -> bool:
        """Not supported on this device.
        """

        return self.is_connected()

    def get_status(self):
        """Not supported on this device.
        """

    def check_errors(self):
        """Not supported on this device.
        """

    def clear_errors(self):
        """Not supported on this device.
        """

    def set_tare(self, weight:float = None):
        """Sets tare weight to the currently measured or the specified value

        Args:
            weight (float, optional): Desired tare weight. Defaults to None.
        """

        if weight is not None:
            # FIXME can be checked later in check_value when f-strings are supported
            if (weight < 0 or weight > self.cmd.MAX_WEIGHT):
                raise PLDeviceCommandError(f"Illegal tare weight {weight} (min - 0 max - {self.cmd.MAX_WEIGHT})")
            self.send(self.cmd.GET_TARE, f"{weight} g")
        else:
            self.send(self.cmd.SET_TARE)

    def get_tare_weight(self) -> float:
        """Gets currently stored tare weight

        Returns:
            float: tare weight in grams
        """

        return self.send(self.cmd.GET_TARE)

    def clear_tare_weight(self):
        """Clears currently stored tare weight
        """

        self.send(self.cmd.CLEAR_TARE)

    def zero_weight(self, stabilize=True):
        """Zeros current balance reading.

        Args:
            stabilize (bool, optional): Whether to wait for the reading to stabilize. Defaults to True.
        """
        if stabilize is True:
            self.send(self.cmd.ZERO)
        else:
            self.send(self.cmd.ZERO_IMMEDIATE)

    @in_simulation_device_returns(42)
    def get_weight(self, stabilize=True) -> float:
        """Gets current weight reading.

        Args:
            stabilize (bool, optional): Whether to wait for the reading to stabilize. Defaults to True.

        Returns:
            float: current weight reading.
        """

        if stabilize is True:
            return self.send(self.cmd.GET_WEIGHT)
        else:
            return self.send(self.cmd.GET_WEIGHT_IMMEDIATE)
