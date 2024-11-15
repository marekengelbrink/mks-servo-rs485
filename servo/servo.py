from dataclasses import dataclass
from enum import Enum
import struct
import minimalmodbus

class Status(Enum):
    IN_1 = 0
    IN_2 = 1
    OUT_1 = 2
    OUT_2 = 3

class MotorType(Enum):
    SERVO_42_D = "Servo42D"
    SERVO_57_D = "Servo57D"


@dataclass
class Servo:
    mb: minimalmodbus.Instrument
    address: int
    max_current: int
    hold_current_percent: int
    microstep: int
    motor_type: MotorType

    def __init__(self, mb: minimalmodbus.Instrument, motor_type: MotorType, address: int, max_current: int, hold_current_percent: int, microstep: int):
        self.mb = mb
        self.address = address
        self.max_current = max_current
        self.hold_current_percent = hold_current_percent
        self.microstep = microstep
        self.motor_type = motor_type

    def read_io(self) -> tuple[bool, bool, bool, bool]:
        """ Read the values of the servo's IO ports """
        io = self.mb.read_registers(functioncode=4, registeraddress=0x34, number_of_registers=1)

        io = io[0]
        return bool(io & 0b1000), bool(io & 0b0100), bool(io & 0b0010), bool(io & 0b0001)


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
