import time
from dataclasses import dataclass
from enum import Enum
import serial
import struct
from crc import Calculator, Crc16
import minimalmodbus


instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
instrument.serial.baudrate = 9600

io = instrument.read_register(0x30, 1)


class Status(Enum):
    IN_1 = 0
    IN_2 = 1
    OUT_1 = 2
    OUT_2 = 3

class MotorType(Enum):
    Servo42D = "Servo42D"
    Servo57D = "Servo57D"

@dataclass
class Servo:
    serial_port: minimalmodbus.Instrument
    address: int
    baudrate: int
    max_current: int
    hold_current_percent: int
    microstep: int
    motor_type: MotorType

    def __init__(self, motor_type: MotorType, address: int, baudrate: int, max_current: int, hold_current_percent: int, microstep: int):
        self.serial_port = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
        self.serial_port.serial.baudrate = baudrate
        self.address = address
        self.baudrate = baudrate
        self.max_current = max_current
        self.hold_current_percent = hold_current_percent
        self.microstep = microstep
        self.motor_type = motor_type

    def read_io(self) -> tuple[bool, bool, bool, bool]:
        """ Read the values of the servo's IO ports """
        io = self.serial_port.read_register(0x30, 1)
        return bool(io & 0b1000), bool(io & 0b0100), bool(io & 0b0010), bool(io & 0b0001)


    def write_io(self, out1: bool, out2: bool) -> None:
        """ Write the specified values to OUT1 and OUT2 IO ports """
        out1_mask = 1
        out2_mask = 1
        out1 = int(out1)
        out2 = int(out2)
        out_values = (out2_mask << 24) | out2 << 16 | out1_mask << 8 | out1
        self.serial_port.write_register(0x36, out_values, 2)

    def motor_calibrate(self) -> None:
        """ Calibrate the servo motor """
        self.serial_port.write_register(0x80, 1, 1)

    def motor_max_current(self) -> None:
        """ Set the motor's maximum current """
        self.serial_port.write_register(0x83, self.max_current, 1)

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
        self.serial_port.write_register(0x9B, hold_current_step, 1)

    def motor_microstep(self) -> None:
        """ Set the motor's microstep """
        self.serial_port.write_register(0x84, self.microstep, 1)

    def go_home(self) -> None:
        """ Move the motor to the home position """
        self.serial_port.write_register(0x91, 1, 1)

    def move_to_absolute_angle(self, acc: int, speed: int, angle: int) -> None:
        """ Move the motor to the specified angle """

        packed_data = struct.pack('>HHi', acc, speed, angle)
        inputs = list(struct.unpack('>HHHH', packed_data))
        self.serial_port.write_registers(0x90, inputs)
