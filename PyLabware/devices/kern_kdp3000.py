"""PyLabware driver for Kern KDP3000 digital weighting platform (balance)."""

import re
from typing import Optional, Union, Dict, Any
import serial

# Core imports
from .. import parsers as parser
from ..controllers import AbstractBalance, in_simulation_device_returns
from ..exceptions import PLConnectionError, PLDeviceCommandError, PLDeviceError
from ..models import LabDeviceCommands, ConnectionParameters


class KDP3000BalanceCommands(LabDeviceCommands):
    """Collection of commands for Kern KDP3000 balance.
    """

        # ##########################  Constants ##################################
    # ID string to check that correct device is connected
    DEFAULT_NAME = "KDP3000"
    # Maximum allowed weight, in grams
    MAX_WEIGHT = 3000
    # Response and error codes. These differ a bit from command to command, unfortunately.
    RESPONSE_CODES = {
        "A": "Acknowledged.",
        "B": "More data to follow.",
        "S": "Stable.",
        "D": "Dynamic."
    }

    ERROR_CODES = {
        "L": "Logical error or invalid parameter.",
        "I": "Internal error.",
        "ES": "Syntax error.",
        "+": "Overweight!",
        "-": "Underweight!"
    }

    # ################### Control commands ###################################
    # Get list of commands
    GET_CMD_LIST = {"name": "I0", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get command interface level and version
    GET_CMD_VER = {"name": "I1", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Get balance name
    GET_NAME = {"name": "I2", "reply":{"type": str, "parser": parser.splitter, "args":[" ", 1]}}
    # Get software version
    GET_SW_VER = {"name": "I3", "reply":{"type": str, "parser": parser.splitter, "args":[" ", 1]}}
    # Get serial number
    GET_SER_NUM = {"name": "I4", "reply":{"type": str, "parser": parser.splitter, "args":[" ", 1]}}
    # Get software id number
    GET_SW_ID = {"name": "I5", "reply":{"type": str, "parser": str.split, "args":[" "]}}
    # Set tare weight
    SET_TARE = {"name": "T", "reply":{"type": str}}
    # Set tare weight immediately
    SET_TARE_IMMEDIATE = {"name": "TI", "reply":{"type": str}}
    # Zero weight
    SET_ZERO = {"name": "Z", "reply":{"type": str}}
    # Zero weight immediately
    SET_ZERO_IMMEDIATE = {"name": "ZI", "reply":{"type": str}}
    # Get stable weight
    GET_WEIGHT = {"name": "S", "reply":{"type": float, "parser": str.split, "args":[" "]}}
    # Get weight immediately
    GET_WEIGHT_IMMEDIATE = {"name": "SI", "reply":{"type": float, "parser": str.split, "args":[" "]}}
    # Record calibration zero point
    CALIBRATE_ZERO = {"name": "JZ", "reply":{"type": str}}
    # Record max weight calibration point
    CALIBRATE_MAX = {"name": "JG", "reply":{"type": str}}
    # Save calibration data
    CALIBRATION_SAVE = {"name": "JS", "reply":{"type": str}}
    # ################### Configuration commands #############################

    # Reset device
    RESET = {"name": "@", "reply": {"type": str}}


class KDP3000Balance(AbstractBalance):
    """
    This provides a Python class for the Kern KDP3000 balance using KERN KCP protocol
    version 1.01 based on the operation manual KDP-BA_IA-e-1710 Version 1.0 2017-11 GB
    """

    def __init__(self, device_name: str, connection_mode: str, address: Optional[str] = None, port: Optional[Union[str, int]] = None):
        """Default constructor
        """

        # Load commands from helper class
        self.cmd = KDP3000BalanceCommands

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

    @in_simulation_device_returns([KDP3000BalanceCommands.DEFAULT_NAME])
    def is_connected(self) -> bool:
        """ Check if the device is connected via GET_NAME command.
        """

        try:
            reply = self.send(self.cmd.GET_NAME)
        except PLConnectionError:
            return False
        return self.cmd.DEFAULT_NAME in reply

    def parse_reply(self, cmd: Dict, reply: Any) -> str:
        """Overloaded method from base class. We need to do chop off command echo and status before geting to the actual reply.
        """

        # Strip reply terminator and prefix
        reply = parser.stripper(reply.body, self.reply_prefix, self.reply_terminator)
        # Split into parts
        command, response_code, *reply = str.split(reply, " ")
        # Analyze response code
        if response_code in self.cmd.ERROR_CODES:
            if response_code in ("L", "I", "+", "-"):
                raise PLDeviceError(self.cmd.ERROR_CODES[response_code])
            elif response_code == "ES":
                raise PLDeviceCommandError(self.cmd.ERROR_CODES[response_code])
        elif response_code in self.cmd.RESPONSE_CODES:
            # All fine
            # Glue reply back into string and strip empty spaces and quotes
            reply = " ".join(reply).strip(" ").strip('"')
            self.logger.debug("Invoking parse_reply() of the base class with argument <%s>", reply)
            return super().parse_reply(cmd, reply)
        else:
            raise PLDeviceError(f"Unknown response code <{response_code}> received from the device!")

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

    def calibrate(self, internal: bool = False) -> bool:
        """Runs the balance calibration according to the manufacturer specifications.
           This is an interactive method requiring user to putting on/off the weights for external calibration.
           The procedure is based on the original user manual page, section 9 "Adjustment", page 14.
           The commands listed there differ from the ones in the latest KCP protocol specification!

        Args:
            internal (bool): Calibrate using internal weight (if available).

        Returns:
            bool: True if calibration has completed successfully.
        """

        if internal is True:
            self.logger.error("This balance doesn't support internal calibration!")
            return

        input("Please insure that the balance are leveled on a hard surface and the weighting pan is empty. Press <Enter> when ready.")
        self.logger.info("Running zero point calibration...")
        try:
            self.send(self.cmd.CALIBRATE_ZERO)
        except PLDeviceError as e:
            self.logger.error("Zero calibration failed! %s", e.args[0])
            return

        input("Please put the calibration weight (3kg, class F1) in the weighting pan and press <Enter>.")
        self.logger.info("Running max point calibration...")
        try:
            self.send(self.cmd.CALIBRATE_MAX)
        except PLDeviceError as e:
            self.logger.error("Maximum weight calibration failed! %s", e.args[0])
            return

        input("Please remove the calibration weight from the weighting pan and press <Enter>.")
        self.logger.info("Storing calibration data...")
        try:
            self.send(self.cmd.CALIBRATION_SAVE)
        except PLDeviceError as e:
            self.logger.error("Storing calibration data failed! %s", e.args[0])
            return
        self.logger.info("Calibration done!")

    def set_tare(self, stable=True) -> None:
        """Sets tare weight to the currently measured value.

        Args:
            stable (bool, optional): Whether to wait for the reading to stable. Defaults to True.
        """

        if stable is True:
            self.send(self.cmd.SET_TARE)
        else:
            self.send(self.cmd.SET_TARE_IMMEDIATE)

    def set_zero(self, stable=True) -> None:
        """Zeros current balance reading.

        Args:
            stable (bool, optional): Whether to wait for the reading to stable. Defaults to True.
        """
        if stable is True:
            self.send(self.cmd.SET_ZERO)
        else:
            self.send(self.cmd.SET_ZERO_IMMEDIATE)

    @in_simulation_device_returns(42)
    def get_weight(self, stable=True) -> float:
        """Gets current weight reading.

        Args:
            stable (bool, optional): Whether to wait for the reading to stable. Defaults to True.

        Returns:
            float: current weight reading.
        """

        if stable is True:
            weight, unit = self.send(self.cmd.GET_WEIGHT)
        else:
            weight, unit = self.send(self.cmd.GET_WEIGHT_IMMEDIATE)

        return (float(weight), unit)