"""PyLabware driver for Metrohm 781 pH/Ion meter."""

from typing import Optional, Union
import serial

# Core imports
from .. import parsers as parser
from ..controllers import LabDevice, in_simulation_device_returns
from ..exceptions import PLConnectionError
from ..models import LabDeviceCommands, ConnectionParameters


class Metrohm781pHMeterCommands(LabDeviceCommands):
    """Collection of command definitions for Metrohm 781 pH/Ion meter.
    """

    # ##########################  Constants ##################################
    # Default firmware version example and regex
    DEFAULT_FW_VER = "5.781.0010"
    FW_VER_REGEX = r'\d\.\d{3}\.\d{4}'
    # Global instrument statuses
    GLOBAL_STATUSES = {"$R": "Idle", "$G": "Running", "$S": "Stopped"}
    # States of the stirrer
    STIRRER_STATUSES = {"ON": True, "OFF": False}

    # ################### Control commands ###################################
    # Get firmware version
    GET_FW_VER = {"name": "&Config.Aux.Prog $Q", "reply": {"type": str, "parser":parser.researcher, "args":[FW_VER_REGEX]}}
    # Get status
    GET_STATUS = {"name": "$D", "reply": {"type": str}}
    # Get primary measured value (pH in pH mode)
    GET_PRIMARY_MEAS = {"name": "&Info.ActualInfo.MeasValue.Primary $Q", "reply": {"type": float, "parser":parser.stripper, "args":['"', '"']}}
    # Get secondary measured value (T in pH mode)
    GET_SECONDARY_MEAS = {"name": "&Info.ActualInfo.MeasValue.Secondary $Q", "reply": {"type": float, "parser":parser.stripper, "args":['"', '"']}}
    # Get stirrer status
    GET_STIRRER_STATUS = {"name": "&Mode.pH.MeasPara.Stirrer.Status $Q", "reply": {"type": str, "parser":parser.stripper, "args":['"', '"']}}
    # Start the stirrer
    START_STIR = {"name": "&Mode.pH.MeasPara.Stirrer.Status \"ON\""}
    # Stop the stirrer
    STOP_STIR = {"name": "&Mode.pH.MeasPara.Stirrer.Status \"OFF\""}
    # Get stirring speed setpoint
    GET_SPEED_SET = {"name": "&Mode.pH.MeasPara.Stirrer.Rate $Q", "reply": {"type": int, "parser":parser.stripper, "args":['"', '"']}}
    # Set stirring speed
    SET_SPEED = {"name": "&Mode.pH.MeasPara.Stirrer.Rate", "type": str, "check":{"values": [f'"{i}"' for i in range(1,16)]}}


#FIXME add abstract class for the pH meter
class Metrohm781pHMeter(LabDevice):
    """
    This provides a Python class for the Metrohm 781 pH/Ion meter
    based on the original operation manual 8.781.1113 06.2004 / jb.
    """

    def __init__(self, device_name: str, connection_mode: str, address: Optional[str], port: Union[str, int]):
        """Default constructor
        """

        # Load commands from helper class
        self.cmd = Metrohm781pHMeterCommands

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
        self.reply_terminator = "\r\r\n"
        self.args_delimiter = " "

    def initialize_device(self):
        """Set default operation mode & reset.
        """

    @in_simulation_device_returns(Metrohm781pHMeterCommands.DEFAULT_FW_VER)
    def is_connected(self) -> bool:
        """ Check if the device is connected via GET_FW_VER command.
        """

        try:
            reply = self.send(self.cmd.GET_FW_VER)
        except PLConnectionError:
            return False
        return reply is not None

    @in_simulation_device_returns("$R.Mode.pH.DriftOk")
    def is_idle(self) -> bool:
        """Returns True if no measurement is active.
        """

        if not self.is_connected():
            return False
        return self.get_status() == self.cmd.GLOBAL_STATUSES["$R"]

    def get_status(self):
        """ Gets global device status.
        """

        status =  self.send(self.cmd.GET_STATUS)
        self.logger.info("Device status: %s", status)
        # Status is the prefix in the reply
        return self.cmd.GLOBAL_STATUSES[status[:2]]

    def check_errors(self):
        """ Gets device errors.
        """

        status =  self.send(self.cmd.GET_STATUS)
        try:
            errors = status[2:].split(";")[1:]
        except IndexError:
            errors = []
        return errors

    def clear_errors(self):
        """Not supported on this device.
        """

    def start(self):
        pass

    def stop(self):
        pass

    def start_stirring(self):
        """Starts stirring.
        """

        self.send(self.cmd.START_STIR)

    def stop_stirring(self):
        """Stops stirring.
        """

        self.send(self.cmd.STOP_STIR)

    def get_speed(self) -> int:
        """Gets current stirring speed.
        """

        return self.send(self.cmd.GET_SPEED_SET)

    def set_speed(self, speed: int):
        """Sets desired speed.
        """

        self.send(self.cmd.SET_SPEED, f"\"{speed}\"")

    def get_ph(self) -> float:
        """Gets current viscosity rend.
        """

        return self.send(self.cmd.GET_PRIMARY_MEAS)

    def get_temperature(self):
        """ Gets temperature (secondary measurement in pH mode)
        """

        return self.send(self.cmd.GET_SECONDARY_MEAS)
