""" This module provides a class for controlling the servo motor """
import math
from dataclasses import dataclass
from enum import Enum
import struct
import minimalmodbus

class Status(Enum):
    """ Enum for the status of the servo motor """
    IN_1 = 0
    IN_2 = 1
    OUT_1 = 2
    OUT_2 = 3


class MotorType(Enum):
    """ Enum for the type of the servo motor """
    SERVO_42_D = "Servo42D"
    SERVO_57_D = "Servo57D"


max_current_dict = {
    MotorType.SERVO_42_D: 3000,
    MotorType.SERVO_57_D: 5200
}


class GoBackToZeroStatus(Enum):
    """ Enum for the status of the go back to zero pin """
    MOVING = 0
    SUCCESS = 1
    FAILURE = 2


class MotorStatus(Enum):
    """ Enum for the status of the motor """
    READ_FAIL = 0
    STOP = 1
    SPEED_UP = 2
    SPEED_DOWN = 3
    FULL_SPEED = 4
    HOMING = 5
    CALIBRATION = 6

class MotorWorkMode(Enum):
    """ Enum for the work mode of the motor """
    CR_OPEN = 0
    CR_CLOSE = 1
    CR_VFOC = 2
    SR_OPEN = 3
    SR_CLOSE = 4
    SR_VFOC = 5


class MotorActiveEnable(Enum):
    """ Enum for the active enable of the motor """
    ACTIVE_LOW = 0
    ACTIVE_HIGH = 1
    ACTIVE_ALWAYS = 2

class MotorDirection(Enum):
    """ Enum for the direction of the motor """
    CLOCKWISE = 0
    COUNTER_CLOCKWISE = 1


class MotorBaudrate(Enum):
    """ Enum for the baudrate of the motor """
    BAUDRATE_9600 = 1
    BAUDRATE_19200 = 2
    BAUDRATE_25000 = 3
    BAUDRATE_38400 = 4
    BAUDRATE_57600 = 5
    BAUDRATE_115200 = 6
    BAUDRATE_256000 = 7


class MotorEndStopActive(Enum):
    """ Enum for the end stop active """
    ACTIVE_LOW = 0
    ACTIVE_HIGH = 1


class MotorZeroMode(Enum):
    """ Enum for the zero mode """
    DISABLE = 0
    DIR_MODE = 1
    NEAR_MODE = 2


class MotorZeroSpeed(Enum):
    """ Enum for the zero speed """
    SLOWEST = 0
    SLOW = 1
    MID = 2
    FAST = 3
    FASTEST = 4


@dataclass
class Servo:
    """ Class for controlling the servo motor """
    mb: minimalmodbus.Instrument
    address: int
    max_current: int
    hold_current_percent: int
    microstep: int
    motor_type: MotorType

    def __init__(self, mb: minimalmodbus.Instrument, motor_type: MotorType, address: int,
                 max_current: int, hold_current_percent: int, microstep: int):
        self.mb = mb
        self.address = address
        self.max_current = min(max_current, max_current_dict.get(motor_type, 0))
        self.hold_current_percent = hold_current_percent
        self.microstep = microstep
        self.motor_type = motor_type

    def read_encoder_value_carry(self) -> tuple[int, int]:
        """ Read the encoder value """
        encoder_value = self.mb.read_registers(functioncode=4, registeraddress=0x30, number_of_registers=3)
        value = encoder_value[2]
        carry = encoder_value[0] << 8 | encoder_value[1]
        return carry, value

    def read_encoder_value(self) -> int:
        """ Read the encoder value """
        encoder_value = self.mb.read_registers(functioncode=4, registeraddress=0x31, number_of_registers=3)
        value = encoder_value[0] << 16 | encoder_value[1] << 8 | encoder_value[2]
        return value

    def read_speed_rpm(self) -> int:
        """ Read the motor speed in RPM """
        speed = self.mb.read_registers(functioncode=4, registeraddress=0x32, number_of_registers=1)
        return speed[0]

    def read_number_of_pulses(self) -> int:
        """ Read the number of pulses """
        pulses = self.mb.read_registers(functioncode=4, registeraddress=0x33, number_of_registers=2)
        pulses = pulses[0] << 16 | pulses[1]
        return pulses

    def read_io(self) -> tuple[bool, bool, bool, bool]:
        """ Read the values of the servo's IO ports """
        io = self.mb.read_registers(functioncode=4, registeraddress=0x34, number_of_registers=1)
        io = io[0]
        return bool(io & 0b1000), bool(io & 0b0100), bool(io & 0b0010), bool(io & 0b0001)

    def read_error_of_angle(self) -> int:
        """ Read the error of the angle """
        error = self.mb.read_registers(functioncode=4, registeraddress=0x35, number_of_registers=2)
        error = error[0] << 16 | error[1]
        return error

    def read_en_pin_status(self) -> bool:
        """ Read the status of the EN pin """
        status = self.mb.read_registers(functioncode=4, registeraddress=0x3A, number_of_registers=1)
        return bool(status[0])

    def read_go_back_to_zero_status(self) -> GoBackToZeroStatus:
        """ Read the status of the go back to zero pin """
        status = self.mb.read_registers(functioncode=4, registeraddress=0x3B, number_of_registers=1)
        return GoBackToZeroStatus(status[0])

    def read_motor_shaft_protection_status(self) -> bool:
        """ Read the status of the motor shaft protection """
        status = self.mb.read_registers(functioncode=4, registeraddress=0x3E, number_of_registers=1)
        return bool(status[0])

    def read_motor_status(self) -> MotorStatus:
        """ Read the status of the motor """
        status = self.mb.read_registers(functioncode=4, registeraddress=0xF1, number_of_registers=1)
        return MotorStatus(status[0])

    def write_io(self, out1: bool, out2: bool) -> None:
        """ Write the specified values to OUT1 and OUT2 IO ports """
        out1_mask = 1
        out2_mask = 1
        out1 = int(out1)
        out2 = int(out2)
        out_val_1 = out1_mask << 8 | out1
        out_val_2 = out2_mask << 8 | out2
        out_values = [out_val_1, out_val_2]
        self.mb.write_registers(registeraddress=0x36, values=out_values)

    def write_release_shaft_protection(self) -> None:
        """ Release the motor shaft protection """
        self.mb.write_register(functioncode=6, registeraddress=0x3D, value=1)

    def write_restore_default_parameters(self) -> None:
        """ Restore the default parameters """
        self.mb.write_register(functioncode=6, registeraddress=0x3F, value=1)

    def write_restart(self) -> None:
        """ Restart the motor """
        self.mb.write_register(functioncode=6, registeraddress=0x41, value=1)

    def write_calibrate(self) -> None:
        """ Calibrate the servo motor """
        self.mb.write_register(functioncode=6, registeraddress=0x80, value=1)

    def write_work_mode(self, mode: MotorWorkMode) -> None:
        """ Set the motor's work mode """
        val_mode = int(mode)
        self.mb.write_register(functioncode=6, registeraddress=0x82, value=val_mode)

    def write_max_current(self) -> None:
        """ Set the motor's maximum current in mA"""
        self.mb.write_register(functioncode=6, registeraddress=0x83, value=self.max_current)

    def write_hold_current(self) -> None:
        """ Set the motor's hold current """
        hold_current_step = math.floor(float(self.hold_current_percent)/10)
        self.mb.write_register(functioncode=6, registeraddress=0x9B, value=hold_current_step)

    def write_subdivision(self) -> None:
        """ Set the motor's microstep """
        self.mb.write_register(functioncode=6, registeraddress=0x84, value=self.microstep)

    def write_active_enable(self, enable: MotorActiveEnable) -> None:
        """ Enable or disable the motor """
        self.mb.write_register(functioncode=6, registeraddress=0x85, value=enable.value)

    def write_direction(self, direction: MotorDirection) -> None:
        """ Set the motor's direction """
        self.mb.write_register(functioncode=6, registeraddress=0x86, value=direction.value)

    def write_auto_turn_off_screen(self, enable: bool) -> None:
        """ Enable or disable the auto turn off screen """
        self.mb.write_register(functioncode=6, registeraddress=0x87, value=int(enable))

    def write_shaft_protection(self, enable: bool) -> None:
        """ Enable or disable the motor shaft protection """
        self.mb.write_register(functioncode=6, registeraddress=0x88, value=int(enable))

    def write_subdivision_interpolation(self, enable: bool) -> None:
        """ Enable or disable the subdivision interpolation """
        self.mb.write_register(functioncode=6, registeraddress=0x89, value=int(enable))

    def write_baudrate(self, baudrate: MotorBaudrate) -> None:
        """ Set the motor's baudrate """
        self.mb.write_register(functioncode=6, registeraddress=0x8A, value=baudrate.value)

    def write_slave_address(self, address: int) -> None:
        """ Set the motor's slave address """
        self.mb.write_register(functioncode=6, registeraddress=0x8B, value=address)

    def write_modbus(self, enable: bool) -> None:
        """ Enable or disable the modbus """
        self.mb.write_register(functioncode=6, registeraddress=0x8E, value=int(enable))

    def write_lock_key(self, enable: bool) -> None:
        """ Enable or disable the lock key """
        self.mb.write_register(functioncode=6, registeraddress=0x8F, value=int(enable))

    def write_zero_axis(self) -> None:
        """ Zero the motor's axis """
        self.mb.write_register(functioncode=6, registeraddress=0x92, value=1)

    def write_serial(self, enable: bool) -> None:
        """ Enable or disable the serial """
        self.mb.write_register(functioncode=6, registeraddress=0x8F, value=int(enable))

    def write_go_home_parameter(self, end_stop_level: MotorEndStopActive, home_dir: MotorDirection,
                                speed: int, enable_end_stop_limit: bool ) -> None:
        """ Set the go home parameter """
        speed_high = speed >> 8
        speed_low = speed & 0xFF

        values = [end_stop_level.value, home_dir.value, speed_high, speed_low, int(enable_end_stop_limit)]
        self.mb.write_registers(registeraddress=0x90, values=values)

    def write_no_limit_go_home_parameter(self, max_return_angle: int, no_switch_go_home: bool,
                                         no_limit_current: int) -> None:
        """ Set the no limit go home parameter """
        angle_high = max_return_angle >> 8
        angle_low = max_return_angle & 0xFF

        values = [angle_high, angle_low, int(no_switch_go_home), no_limit_current]
        self.mb.write_registers(registeraddress=0x94, values=values)

    def write_end_stop_port_remap(self, enable: bool) -> None:
        """ Enable or disable the end stop port remap """
        self.mb.write_register(functioncode=6, registeraddress=0x9E, value=int(enable))

    def write_zero_mode_parameter(self, set_zero: bool, zero_mode: MotorZeroMode,
                                  zero_dir: MotorDirection, zero_speed: MotorZeroSpeed) -> None:
        """ Set the zero mode parameter """
        values = [zero_mode, set_zero, zero_speed, zero_dir]
        self.mb.write_registers(registeraddress=0x9A, values=values)

    def write_single_turn_zero_return_and_position_error_protection(self,
                                                                    position_protection: bool,
                                                                    single_turn_zero_return: bool,
                                                                    time: int,
                                                                    errors: int) -> None:
        """ Enable or disable the position error protection """
        bool_byte = single_turn_zero_return << 1 | position_protection
        values = [bool_byte, time, errors]
        self.mb.write_registers(registeraddress=0x9D, values=values)

    def go_home(self) -> None:
        """ Move the motor to the home position """
        self.mb.write_register(functioncode=6, registeraddress=0x91, value=1)

    def move_to_absolute_angle(self, acc: int, speed: int, angle: int) -> None:
        """ Move the motor to the specified angle """

        packed_data = struct.pack('>HHi', acc, speed, angle)
        inputs = list(struct.unpack('>HHHH', packed_data))
        self.mb.write_registers(0x90, inputs)
