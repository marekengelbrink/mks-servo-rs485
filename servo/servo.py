""" This module provides a class for controlling the servo motor """
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
        self.max_current = max_current
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
        out_values = (out2_mask << 24) | out2 << 16 | out1_mask << 8 | out1
        self.mb.write_register(0x36, out_values, 2)

    def motor_calibrate(self) -> None:
        """ Calibrate the servo motor """
        self.mb.write_register(functioncode=6, registeraddress=0x80, value=1)

    def motor_max_current(self) -> None:
        """ Set the motor's maximum current """
        self.mb.write_register(0x83, self.max_current, 1)

    def motor_hold_current(self) -> None:
        """ Set the motor's hold current """
        hold_current_step = 0

        if self.hold_current_percent > 10:
            hold_current_step = 1
        if self.hold_current_percent > 20:
            hold_current_step = 2
        if self.hold_current_percent > 30:
            hold_current_step = 3
        if self.hold_current_percent > 40:
            hold_current_step = 4
        if self.hold_current_percent > 50:
            hold_current_step = 5
        if self.hold_current_percent > 60:
            hold_current_step = 6
        if self.hold_current_percent > 70:
            hold_current_step = 7
        if self.hold_current_percent > 80:
            hold_current_step = 8
        self.mb.write_register(0x9B, hold_current_step, 1)

    def motor_microstep(self) -> None:
        """ Set the motor's microstep """
        self.mb.write_register(0x84, self.microstep, 1)

    def go_home(self) -> None:
        """ Move the motor to the home position """
        self.mb.write_register(0x91, 1, 1)

    def move_to_absolute_angle(self, acc: int, speed: int, angle: int) -> None:
        """ Move the motor to the specified angle """

        packed_data = struct.pack('>HHi', acc, speed, angle)
        inputs = list(struct.unpack('>HHHH', packed_data))
        self.mb.write_registers(0x90, inputs)
