"""PyLabware driver for IMI Norgren Cadent 3 syringe pump with integrated valve."""

from typing import Optional, Union, Dict, Any
from time import sleep
import serial
import ast

# Core import
from .. import parsers as parser
from ..controllers import (AbstractSyringePump,
                           AbstractDistributionValve,
                           in_simulation_device_returns)
from ..exceptions import (PLConnectionError, PLDeviceCommandError,
                          PLDeviceError, PLDeviceInternalError,
                          PLDeviceReplyError)
from ..models import LabDeviceCommands, LabDeviceReply, ConnectionParameters


class Cadent3SyringePumpCommands(LabDeviceCommands):
    """Collection of command definitions for Cadent 3 pump, DT protocol based on the manual HUM023 Rev. 2.
    """

    # ################### Configuration constants #############################

    # Mapping of rotary switch settings to apparent pump address on serial
    # Please note that switch position 0 is reserved for controller (see p.22 of the manual)
    # Please note that when using address "all" the individual devices do not provide status responses to commands. (see p.23 of the manual)
    SWITCH_ADDRESSES = {
        "1": "1",
        "2": "2",
        "3": "3",
        "4": "4",
        "5": "5",
        "6": "6",
        "7": "7",
        "8": "8",
        "9": "9",
        "A": ":",
        "B": ";",
        "C": "<",
        "D": "=",
        "E": ">",
        "F": "?",
        "all": "_"
    }

    # Allowed valve positions
    VALVE_POSITIONS = range(1,16)

    # Valve types for Uxx command
    VALVE_TYPES = {
        "3PORT_NDISTR": "1",  # 3 way non-distribution valve
        "3PORT_DISTR": "2",  # 3 way distribution valve
        "4PORT_NDISTR": "3",  # 4 way non-distribution valve
        "4PORT_DISTR": "4",  # 4 way distribution valve
        "5PORT_NDISTR": "5",  # 5 way non-distribution valve
        "5PORT_DISTR": "6",  # 5 way distribution valve
        "6PORT_NDISTR": "7",  # 6 way non-distribution valve
        "6PORT_DISTR": "14",  # 6 way distribution valve
        "8PORT_NDISTR": "9",  # 8 way non-distribution valve
        "8PORT_DISTR": "10",  # 8 way distribution valve
        "12PORT_DISTR": "11",  # 12 way non-distribution valve
        "2PORT_DISTR": "13",  # 2 way distribution valve
        "2PORT_SOL": "15",  # 2 way FAS solenoid valve
        "NONE": "20202" # no valve installed
    }

    # ## Cadent 3 error codes ###
    # Error codes are represented as a bit field occupying 5 right-most bits of status byte (see p. 85 of the manual)
    ERROR_CODES = {
        0b00000: "no error",
        0b00001: "syringe failed to initialize",
        0b00010: "invalid command",
        0b00011: "invalid argument",
        0b00100: "communication error",
        0b00101: "invalid \"R\" command",
        0b00110: "supply voltage too low",
        0b00111: "device not initialized",
        0b01000: "program in progress",
        0b01001: "syringe overload",
        0b01010: "valve overload",
        0b01011: "syringe move not allowed",
        0b01100: "cannot move against limit",
        0b01111: "command buffer overflow",
        0b10000: "use for 3-way valve only",
        0b10001: "loops nested too deep",
        0b10010: "program label not found",
        0b10011: "end of program not found",
        0b10100: "out of program space",
        0b10101: "home not set",
        0b10110: "too many program calls",
        0b10111: "program not found",
        0b11000: "valve position error",
        0b11001: "syringe position corrupted",
        0b11010: "syringe may go past home"
    }

    # Default status - pump initialized, idle, no error
    DEFAULT_STATUS = "`"

    # ################### Control commands ###################################

    # ## Initialization commands ##
    INIT_ALL = {"name": "W4", "reply": {"type": str}}
    # Initialize plunger & valves, valve numbering - CW from syringe (first on the left)
    # For non-distribution valves - set valve to the right
    INIT_ALL_Z = {"name": "Z4", "reply": {"type": str}}
    # Initialize plunger & valves, valve numbering - CCW from syringe (first on the right)
    # For non-distribution valves - set valve to the left
    INIT_ALL_Y = {"name": "Y4", "reply": {"type": str}}
    # Initialize syringe only
    INIT_SYRINGE = {"name": "W10", "reply": {"type": str}}
    # Initialize valve only
    INIT_VALVE = {"name": "W7", "reply": {"type": str}}
    # Reset controller
    RESET_CONTROLLER = {"name": "W9", "reply": {"type": str}}

    # ## Plunger movement commands ##
    # Move plunger to absolute position
    SYR_MOVE_ABS = {"name": "A", "type":int, "reply": {"type": str}}
    # Move plunger to absolute position, do not set busy flag
    SYR_MOVE_ABS_NOBUSY = {"name": "a", "type": int, "reply": {"type": str}}
    # Relative pick-up (aspirate)
    SYR_SUCK_REL = {"name": "P", "type": int, "reply": {"type": str}}
    # Relative pick-up (aspirate), do not set busy flag 
    SYR_SUCK_REL_NOBUSY = {"name": "p", "type": int, "reply": {"type": str}}
    # Relative dispense
    SYR_SPIT_REL = {"name": "D", "type": int, "reply": {"type": str}}
    # Relative dispense, do not set busy flag
    SYR_SPIT_REL_NOBUSY = {"name": "d", "type": int, "reply": {"type": str}}

    # ## Valve movement commands ##

    # Moves the valve to the position selected by n
    VALVE_MOVE = {"name": "o", "type": str, "reply": {"type": str}}

    # below commands only used with a three-way non-distribution valve!!!
    # Move valve to input port using the shortest path.
    VALVE_MOVE_I = {"name": "I", "check": {"values": VALVE_POSITIONS}, "reply": {"type": str}}
    # Move valve to output port using shortest path. For rotary valves there must be a port designated as the output.
    VALVE_MOVE_O = {"name": "O", "check": {"values": VALVE_POSITIONS}, "reply": {"type": str}}
    # Rotate valve to bypass position. No check as there are no arguments.
    VALVE_MOVE_B = {"name": "B", "reply": {"type": str}}

    # ## Execution flow control commands ##
    # Execute command string
    PRG_RUN = {"name": "R", "reply": {"type": str}}
    # Repeat last command string (background only)
    PRG_RPT_LAST = {"name": "X", "reply": {"type": str}}
    # Store program string into EEPROM
    PRG_EEPROM_ST = {"name": "E", "reply": {"type": str}}
    # Load and run stored user script n in background (n: 1...99)
    PRG_EEPROM_EXEC = {"name": "r", "reply": {"type": str}}
    # Mark start of looped command sequence
    PRG_MARK_LOOP_START = {"name": "g"}
    # Mark end of looped command sequence
    PRG_MARK_LOOP_END = {"name": "G"}
    # Delay command execution n milliseconds
    PRG_DELAY_EXEC = {"name": "M"}
    # Halt command execution (wait for R command and/or ext. input change)
    PRG_HALT = {"name": "H", "reply": {"type": str}}
    # Terminate commands execution or background script
    PRG_TERM = {"name": "T", "reply": {"type": str}}

    # ## Report commands ##
    # Query pump status, return ASCII rendition of binary code
    GET_STATUS = {"name": "Q", "reply": {"type": str}}
    # Query firmware version
    GET_FW_VER = {"name": "?32", "reply": {"type": str}}
    # Query plunger absolute position in steps (reply: ‘?’ if invalid, else syringe position)
    GET_SYR_POS = {"name": "?", "reply": {"type": int}}
    # Query start velocity [steps/sec]
    GET_START_VEL = {"name": "?1", "reply": {"type": int}}
    # Query maximum velocity [steps/sec]
    GET_MAX_VEL = {"name": "?2", "reply": {"type": int}}
    # Query cut-off velocity [steps/sec]
    GET_STOP_VEL = {"name": "?3", "reply": {"type": int}}
    # Query acceleration/deceleration ramp [steps/step]
    GET_STEP_RAMP = {"name": "?30", "reply": {"type": str, "parser": parser.splitter, "args": [",", 0, 2]}}
    # Query backlash increments setting
    GET_BACK_INC = {"name": "?31", "reply": {"type": str}}
    # Query stepper motor supply voltage [200mV units]
    GET_VOLT = {"name": "*", "reply": {"type": int}}
    # Query valve position as port (reply: 1…16 = valve port number, ? = Invalid position)
    GET_VALVE_POS = {"name": "?8", "reply": {"type": str, "parser": str.upper}}
    # Query flag valve initialized
    GET_VALVE_INIT = {"name": "f65?", "reply": {"type": bool}}
    # Query flag syringe pump initialized
    GET_SYR_INIT = {"name": "f66?", "reply": {"type": bool}}

    # ################### Configuration commands #############################

    # ## Configuration commands ##
    # Save current operational parameters in non-volatile configuration parameters
    SAVE_TO_NVM = {"name": "!", "reply":{"type": str}}
    # Get/Set valve type
    GET_SET_VALVE_TYPE = {"name": "~V", "type": str, "reply": {"type": str}}
    # Set the valve position which is used by the Yn command
    SET_VALVE_Y_POS = {"name": "~Y", "type": str, "reply": {"type": str}}
    # Set the valve position which is used by the Zn command
    SET_VALVE_Z_POS = {"name": "~Z", "type": str, "reply": {"type": str}}

    # ## Indirect Variables ##
    # Syringe pump resolution (half steps) [rw]
    GET_SET_SYR_RES = {"name": "?@26", "reply": {"type": int}}

    # ## Motion Variable Command Definitions ##
    # Set start velocity (beginning of ramp)
    SET_START_VEL = {"name": "v", "type": int, "check": {"min": 1, "max": 10000}, "reply": {"type": str}}
    # Set maximum velocity (top of ramp) [steps/sec]
    SET_MAX_VEL = {"name": "V", "type": int, "check": {"min": 1, "max": 10000}, "reply": {"type": str}}
    # Set cut-off velocity (end of ramp)
    SET_STOP_VEL = {"name": "c", "type": int, "check": {"min": 1, "max": 10000}, "reply": {"type": str}}  
    # Set syringe acceleration slope in n step increments/step
    # Note: Datasheet claims this sets acceleration AND deceleration slope, but it's not true
    SET_ACEL_RAMP_SLOPE = {"name": "L", "type": int, "reply": {"type": str}}
    # Set syringe deceleration slope in n step decrements/step.
    SET_DEC_RAMP_SLOPE = {"name": "l", "type": int, "reply": {"type": str}}
    
    # ## Pump I/O Commands ##
    # Set external output
    SET_EXT_OUT = {"name": "U", "reply": {"type": str}}
    # Reset external output
    RESET_EXT_OUT = {"name": "u", "reply": {"type": str}}


class Cadent3SyringePump(AbstractSyringePump, AbstractDistributionValve):
    """
    This provides a Python class for the IMI Norgren Cadent 3 syringe pump
    based on the the original operation manual HUM023 Rev. 2.
    """

    # As there can be multiple pumps on the same serial port, it's necessary to maintain a list
    # Example: {"COM3":{"pumps":1, "conn":<PyLabware.connections.SerialConnection at 0x1111>}}
    BUS_DEVICES = {}

    def __init__(self, device_name: str, connection_mode: str, address: Optional[str], port: Union[str, int],
                 switch_address: Union[int, str], syringe_size: Optional[float] = None, valve_type: str = "3PORT_NDISTR"):
        """Default constructor.
        """

        # Load commands from helper class
        self.cmd = Cadent3SyringePumpCommands

        # Flag to indicate that explicit volumetric calibration
        # (comparing dispensed volume to required volume) has been performed
        self._volumetric_calibrated = False
        # Size of the syringe installed, in mL
        self._syringe_size = syringe_size
        # Calibration factor
        self._steps_per_ml = None

        # Check that valid valve type has been passed
        try:
            self._valve_type = Cadent3SyringePumpCommands.VALVE_TYPES[valve_type]
        except KeyError:
            raise PLDeviceError("Invalid valve type <{}> provided!".format(valve_type)) from None

        # Connection settings
        connection_parameters: ConnectionParameters = {}
        # TCP/IP relevant settings
        connection_parameters["port"] = port
        # Save port separately to access reference count in connect()/disconnect() later
        self.port = port
        connection_parameters["address"] = address
        # RS-232/RS-485 relevant settings
        connection_parameters["baudrate"] = 9600
        connection_parameters["bytesize"] = serial.EIGHTBITS
        connection_parameters["parity"] = serial.PARITY_NONE

        super().__init__(device_name, connection_mode, connection_parameters)

        # Check if we already have the required COM port open
        try:
            self.connection = type(self).BUS_DEVICES[port]["conn"]
            # Increase reference count
            self.logger.debug("Existing connection for port <%s> found, reusing <%s>", port, self.connection)
            type(self).BUS_DEVICES[port]["pumps"] += 1
        except KeyError:
            type(self).BUS_DEVICES[port] = {"pumps": 1, "conn": self.connection}
            self.logger.debug("Storing new serial connection reference <%s> for port <%s>", self.connection, port)

        # Set switch address
        try:
            switch_address = self.cmd.SWITCH_ADDRESSES[str(switch_address)]
        except KeyError:
            raise PLDeviceError("Invalid switch address <{}> supplied!".format(switch_address)) from None

        # Protocol settings
        self.command_prefix = "/" + switch_address
        # Run commands after sending them to pump by default (R appended)
        self.command_terminator = self.cmd.PRG_RUN["name"] + "\r"
        self.reply_prefix = "/0"
        self.reply_terminator = "\x03\r\n" # note that terminator is "\x03\r\n\xff" but "\xff" is dropped bc. we cannot parse it
        self.args_delimiter = ""
        # Pump status byte
        self._last_status = 0

    @property
    def autorun(self):
        """Property showing if the commands should be executed immediately,
        or queued instead.
        """

        return self.cmd.PRG_RUN["name"] in self.command_terminator

    @autorun.setter
    def autorun(self, value):
        """Setter for the autorun property.
        """

        if value is True:
            self.command_terminator = self.cmd.PRG_RUN["name"] + "\r"
        else:
            self.command_terminator = "\r"

    @property
    def syringe_size(self):

        if self._syringe_size is None:
            raise PLDeviceCommandError("Syringe size not set! Functions with volumetric dosing are not available! Please set either step_per_ml or syringe_size properties.")
        return self._syringe_size

    @syringe_size.setter
    def syringe_size(self, value):

        if self._volumetric_calibrated:
            self.logger.warning("Changing syringe size resetted volumetric calibration.")
        self._syringe_size = value
        self._volumetric_calibrated = False
        self.steps_per_ml = 6000 / value

    @property
    def steps_per_ml(self):

        if self._steps_per_ml is None:
            raise PLDeviceCommandError("Calibration factor not set! Functions with volumetric dosing are not available! Please set either step_per_ml or syringe_size properties.")
        return self._steps_per_ml

    @steps_per_ml.setter
    def steps_per_ml(self, value):

        self._steps_per_ml = int(value)

    def connect(self):
        """ Checks whether the connection has been already opened, if not - opens it.
        """

        if not self.connection.is_connection_open():
            return super().connect()
        self.logger.info("Connection already open.")

    def disconnect(self):
        """ Checks whether we are the last device using this port. If yes - closes the connection.
            If not - decreases the reference count.
        """

        # TODO check if we are closing the connection for ourselves!
        if not self.connection.is_connection_open():
            self.logger.warning("Connection not open yet!")
            return
        # Check current reference count for the serial port open
        #FIXME probably wouldn't work for other than serial connection
        current_refs = type(self).BUS_DEVICES[self.port]["pumps"]
        current_refs -= 1
        if current_refs == 0:
            _ = type(self).BUS_DEVICES.pop(self.port)
            super().disconnect()
        else:
            type(self).BUS_DEVICES[self.port]["pumps"] = current_refs
            self.logger.info("%s more devices left on the bus, leaving connection open.", current_refs)

    def parse_reply(self, cmd: Dict, reply: Any) -> str:
        """Overloaded method from base class. We need to do some more
        complex processing here for the status byte manipulations.
        """

        # Strip reply terminator and prefix
        reply = parser.stripper(reply.body, self.reply_prefix, self.reply_terminator)
        # Then analyze status byte
        # Status byte is the 1st byte of reply string, & we need it's byte code.
        self._last_status = ord(reply[0])
        self.check_errors()
        self.logger.debug("parse_reply()::status byte checked, invoking parsing on <%s>", reply[1:])
        # Chop off status byte & do standard processing
        return super().parse_reply(cmd, reply[1:])

    def check_errors(self):
        """Checks error bits in the status byte of the pump reply.
        """

        self.logger.debug("check_errors()::checking errors on byte <%s>", self._last_status)
        # Error code is contained in 5 right-most bits,
        # so we need to chop off the rest
        error_code = self._last_status & 0b11111
        # No error
        if error_code == 0:
            return None
        try:
            raise PLDeviceInternalError(self.cmd.ERROR_CODES[error_code])
        except KeyError:
            # This shouldn't really happen, means that pump replied with
            # error code not in the ERROR_CODES dictionary
            # (which completely copies the manual)
            raise PLDeviceReplyError("Unknown error! Status byte: {}".format(bin(self._last_status))) from None

    def is_connected(self) -> bool:
        """Checks whether the device is connected by
        checking it's firmware version.
        """

        try:
            self.autorun = False
            version = self.send(self.cmd.GET_FW_VER)
            self.logger.debug("is_connected()::Device connected; FW version <%s>", version)
            return True
        except PLConnectionError:
            return False
        finally:
            self.autorun = True

    def get_status(self):
        """ Returns status byte of the pump.
        """

        self.autorun = False
        current_status = self.send(self.cmd.GET_STATUS)
        self.autorun = True
        return current_status

    def clear_errors(self):
        """Happens automatically upon errors read-out,
        except those requiring pump re-initialization.
        """

    def initialize_device(self, init_pos="in", input_port=None, output_port=None):
        """Runs pump initialization. If input/output ports are provided,
        input is set on Y variable, output on Z. Init position is the position of the valve
        befor the plunger is moved. Default is input or A position if none given.
        """

        # Select appropriate command depending on the direction
        if init_pos == "in":
            cmd = self.cmd.INIT_ALL_Y
        elif init_pos == "out":
            cmd = self.cmd.INIT_ALL_Z
        else:
            raise PLDeviceCommandError("Invalid direction for valve initialization provided!")

        # Check if we are asked to use specific input/output ports.
        # Set Y and Z positions accordingly.
        self.autorun = False
        if input_port is not None:
            try:
                if int(input_port) not in self.cmd.VALVE_POSITIONS:
                    raise ValueError
                self.send(self.cmd.SET_VALVE_Y_POS, str(input_port))
            except ValueError:
                raise PLDeviceCommandError("Invalid port for initialization was provided!")
        if output_port is not None:
            try:
                if int(output_port) not in self.cmd.VALVE_POSITIONS:
                    raise ValueError
                self.send(self.cmd.SET_VALVE_Z_POS, str(output_port))
            except ValueError:
                raise PLDeviceCommandError("Invalid port for initialization was provided!")
        self.autorun = True
        # Send commands & check errors in the reply
        self.send(cmd)
        self.wait_until_ready()
        self.logger.info("Device initialized.")

    @in_simulation_device_returns(True)
    def is_initialized(self) -> bool:
        """Check if pump has been initialized properly after power-up.
           Note that this is usually the case and pump needs no re-initialization on power-up.
           According to IMI Engineering: Flags 65 and 66 may not work as expected.
           Use move command and check the status byte instead.
        """
        try:
            self.autorun = False
            valve_init = self.send(self.cmd.GET_VALVE_INIT)
            syringe_init = self.send(self.cmd.GET_SYR_INIT)
        except PLConnectionError:
            return False
        finally:
            self.autorun = True
        # Busy/idle bit is 6th bit of the status byte. 0 - busy, 1 - idle
        if self._last_status & 1 << 5 == 0:
            self.logger.debug("is_idle()::false.")
            return False
        self.logger.debug("is_idle()::true.")
        return valve_init and syringe_init

    @in_simulation_device_returns(LabDeviceReply(body=Cadent3SyringePumpCommands.DEFAULT_STATUS))
    def is_idle(self) -> bool:
        """Checks if pump is in idle state.
        """

        try:
            self.autorun = False
            _ = self.send(self.cmd.GET_STATUS)
        except PLConnectionError:
            return False
        finally:
            self.autorun = True
        # Busy/idle bit is 6th bit of the status byte. 0 - busy, 1 - idle
        if self._last_status & 1 << 5 == 0:
            self.logger.debug("is_idle()::false.")
            return False
        self.logger.debug("is_idle()::true.")
        return True

    def start(self):
        """Starts program execution."""

        if self.autorun is True:
            self.logger.warning("Sending run command with autorun enabled is not required.")
            return
        self.send(self.cmd.PRG_RUN)

    def stop(self):
        """ Stops executing current program/action immediately."""

        self.send(self.cmd.PRG_TERM)

    def set_speed(self, speed: int):
        """ Sets maximum velocity (top of the ramp) for the syringe motor.
            Velocity can be any value between 1 and 10000.
        """

        self.set_max_velocity(speed)

    def get_speed(self):
        raise NotImplementedError("Getting speed is not supported on this model.")

    def move_home(self):
        self.move_plunger_absolute(0)

    def move_plunger_absolute(self, position: int, set_busy: bool = True):
        """Makes absolute plunger move.
        """

        if set_busy is True:
            cmd = self.cmd.SYR_MOVE_ABS
        else:
            cmd = self.cmd.SYR_MOVE_ABS_NOBUSY
        # Send command & check reply for errors
        self.execute_when_ready(self.send, cmd, position)

    def get_plunger_position(self) -> int:
        """Returns absolute plunger position.
        """

        # Send command & check reply for errors
        # If autorun is not disabled for the ? command, pump reports an operand error
        autorun_state = self.autorun
        self.autorun = False
        try:
            position = self.send(self.cmd.GET_SYR_POS)
        finally:
            self.autorun = autorun_state
        return position

    def move_plunger_relative(self, position: int, set_busy: bool = True):
        """Makes relative plunger move. This is a wrapper for
        dispense()/withdraw().
        """

        position = int(position)
        if position > 0:
            return self.withdraw(increments=position, set_busy=set_busy)
        return self.dispense(increments=abs(position), set_busy=set_busy)

    def dispense(self, volume_ml: float = None, increments: int = None, set_busy: bool = True):
        """Makes relative dispense. If increments are not provided, they are calculated from the volume.
        """

        if increments is None:
            if volume_ml is None:
                raise PLDeviceCommandError("Either increments or volume must be provided!")
            increments = volume_ml * self.steps_per_ml
        if set_busy is True:
            cmd = self.cmd.SYR_SPIT_REL
        else:
            cmd = self.cmd.SYR_SPIT_REL_NOBUSY

        # Send command & check reply for errors
        self.execute_when_ready(self.send, cmd, increments)

    def withdraw(self, volume_ml: float = None, increments: int = None, set_busy: bool = True):
        """Makes relative aspiration. If increments are not provided, they are calculated from the volume.
        """

        if increments is None:
            if volume_ml is None:
                raise PLDeviceCommandError("Either increments or volume must be provided!")
            increments = volume_ml * self.steps_per_ml
        if set_busy is True:
            cmd = self.cmd.SYR_SUCK_REL
        else:
            cmd = self.cmd.SYR_SUCK_REL_NOBUSY
        # Send command & check reply for errors
        self.execute_when_ready(self.send, cmd, increments)

    def prime_pump(self, port: str, cycles: int = 2, increments: int = 2000) -> None:
        """ Primes the tubing and syringe to displace air
        """

        self.execute_when_ready(self.set_valve_position, port)
        self.execute_when_ready(self.move_plunger_absolute, 0)
        for c in range(cycles):
            self.logger.info("Priming the pump <%s>, port %s, cycle %s out of %s...", self.device_name, port, c+1, cycles)
            # 2000 - 1/3 full stroke
            self.execute_when_ready(self.move_plunger_absolute, increments)
            # Allow the liquid to settle down. Adequate speed should be used.
            sleep(3)
            self.execute_when_ready(self.move_plunger_absolute, 0)
            self.wait_until_ready()
            self.logger.info("Priming cycle %s done", c+1)
        self.wait_until_ready()
        self.logger.info("Priming done.")

    def transfer(self, port_from: str, port_to: str, volume_ml):
        """ Transfers the required amount in mL from <port_from> to <port_to>.
        """

        increments = volume_ml * self.steps_per_ml
        complete_strokes = round(increments // 6000)
        remainder = increments % 6000
        self.logger.info("Executing transfer of <%s> mL from <%s> to <%s>", volume_ml, port_from, port_to)
        self.logger.debug("Calculated <%s> full strokes plus <%s> mL", complete_strokes, remainder/self.steps_per_ml)

        # Do full strokes
        for i in range(complete_strokes):
            self.logger.info("Doing full transfer cycle <%s> of <%s>", i+1, complete_strokes)
            self.execute_when_ready(self.set_valve_position, port_from)
            sleep(1)
            self.execute_when_ready(self.move_plunger_absolute, 6000)
            sleep(3)
            self.execute_when_ready(self.set_valve_position, port_to)
            sleep(1)
            self.execute_when_ready(self.move_plunger_absolute, 0)

        # Do the remainder
        if remainder != 0:
            self.logger.info("Transferring remaining <%s> mL", remainder/self.steps_per_ml)
            self.execute_when_ready(self.set_valve_position, port_from)
            sleep(1)
            self.execute_when_ready(self.move_plunger_absolute, remainder)
            sleep(3)
            self.execute_when_ready(self.set_valve_position, port_to)
            sleep(1)
            self.execute_when_ready(self.move_plunger_absolute, 0)

        # Wait for the final dispense to finish
        self.wait_until_ready()

        self.logger.info("Transfer done.")

    def calibrate_volume(self):
        """ Runs interactive volume calibration to set steps_per_ml coefficient correctly.
        """

        port_from = None
        port_to = None
        volume_measured = None
        calibration_steps = 2000

        print("Starting interactive volume calibration. Please check that the syringe is empty.")
        while port_from not in self.cmd.VALVE_POSITIONS:
            port_from = input("Please enter the port to withdraw the liquid from (1-9): ")
        while port_to not in self.cmd.VALVE_POSITIONS:
            port_to = input("Please enter the port to dispense the liquid to (1-9): ")
        port_to = "I" + port_to
        port_from = "I" + port_from
        print("Priming...")
        self.execute_when_ready(self.prime_pump, port_from)
        print("Priming done.")
        print("Withdrawing...")
        self.execute_when_ready(self.move_plunger_absolute, calibration_steps)
        sleep(3)
        self.execute_when_ready(self.set_valve_position, port_to)
        print("Dispensing...")
        self.execute_when_ready(self.move_plunger_absolute, 0)
        while volume_measured is None:
            try:
                volume_measured = float(input("Please enter the exact volume of the liquid dispensed, mL: "))
            except ValueError:
                pass
        # Set calibration factor
        self.steps_per_ml = int(calibration_steps / volume_measured)
        # If syringe volume was not set manually - back calculate it from the calibration factor
        if self._syringe_size is None:
            self.syringe_size = 6000 / self.steps_per_ml
        print(f"Calibration done. Calibration factor (steps_per_ml): {self.steps_per_ml}. Calculated syringe volume: {self.syringe_size:.2f} mL.")
        self._volumetric_calibrated = True

    def set_valve_position(self, requested_position: int|str, override_direction: str = None):
        """ Moves the distribution valve to the requested position on the shortest path 
            or in the direction specified by override_direction.
        """
        # select appropriate modifier if override_direction is provided
        if override_direction is None:
            dir_modifier = ""
        elif override_direction == "CW":
            dir_modifier = "+"
        elif override_direction == "CCW":
            dir_modifier = "-"
        else:
            raise PLDeviceCommandError("Invalid override direction provided!")
        try:
            if int(requested_position) not in self.cmd.VALVE_POSITIONS:
                raise ValueError
            requested_position = str(requested_position)
        except ValueError:
            raise PLDeviceCommandError(f"Unknown valve position <{requested_position}> requested!")
        args = dir_modifier+requested_position
        # Send command & check reply for errors
        self.execute_when_ready(self.send, self.cmd.VALVE_MOVE, args)

    def get_valve_position(self) -> str:
        """Reads current position of the valve.
        """

        # Send command & check reply for errors
        return self.send(self.cmd.GET_VALVE_POS)

    def set_ramp_slope(self, accel_slope: int, decel_slope: int):
        """Sets slope of acceleration/deceleration ramp for the syringe motor.
        """

        # Send command & check reply for errors
        self.send(self.cmd.SET_ACEL_RAMP_SLOPE, accel_slope)
        self.send(self.cmd.SET_DEC_RAMP_SLOPE, decel_slope)

    def get_ramp_slope(self) -> tuple[int, int]:
        """ Reads slope of acceleration/deceleration ramp for the syringe motor.
            Returns tuple of two integers: acceleration slope, deceleration slope.
        """
        
        self.autorun = False
        slope = self.send(self.cmd.GET_STEP_RAMP)
        self.autorun = True
        return tuple([int(s.strip()) for s in slope])
    
    def set_start_velocity(self, start_velocity: int):
        """Sets starting velocity for the syringe motor.
        """

        # Send command & check reply for errors
        self.send(self.cmd.SET_START_VEL, start_velocity)
    
    def set_max_velocity(self, max_velocity: int):
        """Sets maximum velocity for the syringe motor.
        """ 

        # Send command & check reply for errors
        self.send(self.cmd.SET_MAX_VEL, max_velocity)

    def set_stop_velocity(self, stop_velocity: int):
        """Sets stopping velocity for the syringe motor.
        """

        # Send command & check reply for errors
        self.send(self.cmd.SET_STOP_VEL, stop_velocity)

    def get_velocities(self) -> tuple[int, int, int]:
        """ Reads velocity settings for the syringe motor.
            Returns tuple of three integers: start velocity, maximum velocity, stop velocity.
        """

        # Send command & check reply for errors
        self.autorun = False
        start = self.send(self.cmd.GET_START_VEL)
        max = self.send(self.cmd.GET_MAX_VEL)
        stop = self.send(self.cmd.GET_STOP_VEL)
        self.autorun = True
        return int(start), int(max), int(stop)

        self.autorun = False
        start_vel = self.send(self.cmd.GET_START_VEL)
        max_vel = self.send(self.cmd.GET_MAX_VEL)
        stop_vel = self.send(self.cmd.GET_STOP_VEL)
        self.autorun = True
        return int(start_vel), int(max_vel), int(stop_vel)

    def get_valve_type(self) -> str:
        """Reads valve type.
        """

        # Send command & check reply for errors
        self.autorun = False
        answer = self.send(self.cmd.GET_SET_VALVE_TYPE)
        self.autorun = True
        return answer
    
    def set_valve_type(self, valve_type: str, confirm: bool = False):
        """Sets valve type. This command requires power-cycle to activate new settings!
        """
        try:
            # If valve_type is exactly 5 digits, it's considered a IMI Norgren valve part number
            if valve_type.isdigit() and len(valve_type) == 5:
                self._valve_type = valve_type
            else:
                # Get correct valve code from VALVE_TYPES dictionary
                self._valve_type = Cadent3SyringePumpCommands.VALVE_TYPES[valve_type]
        except KeyError:
            raise PLDeviceCommandError("Invalid valve type requested!")
        self.autorun = False
        # Send command & check reply for errors
        self.send(self.cmd.GET_SET_VALVE_TYPE, self._valve_type)
        if confirm is not True:
            self.logger.info("Please, execute set_valve_type(valve_type, confirm=True)"
                                "to write new valve configuration to pump NMV.")
        else:
            self.send(self.cmd.SAVE_TO_NVM)
        self.autorun = True

    def get_pump_configuration(self):
        """Reads pump EEPROM configuration.
        """

        # FIXME: This is supported but not implemented yet
        raise NotImplementedError("Reading pump configuration is not supported yet.")
    

    def get_pump_resolution(self) -> int:
        """ Returns the syringe pump resolution.
            The driver is available in three resolutions: 6000 half-steps, 
            12000 half-steps and 24000 half-steps for the same 3cm stroke length.
        """
        self.autorun = False
        res = self.send(self.cmd.GET_SET_SYR_RES)
        self.logger.debug("get_pump_resolution()::Syringe Pump resolution <%i>", res)
        self.autorun = True
        return res
