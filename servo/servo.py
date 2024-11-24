""" This module provides a class for controlling the servo motor """
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
import struct
import minimalmodbus

ENCODER_STEPS = 16384
ANGLE_TO_AXIS = ENCODER_STEPS/360

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

class MotorSpeedParameterSaveClean(Enum):
    """ Enum for the speed parameter save clean """
    SAVE = 0xC8
    CLEAN = 0xCA

@dataclass
class Servo:
    """ Class for controlling the servo motor """
    mb: minimalmodbus.Instrument
    address: int
    max_current: int
    hold_current_percent: int
    full_steps: int
    micro_steps: int
    motor_type: MotorType

    def __init__(self, mb: minimalmodbus.Instrument, motor_type: MotorType, address: int,
                 max_current: int, hold_current_percent: int,full_steps: int, micro_steps: int):
        self.mb = mb
        self.address = address
        self.max_current = min(max_current, max_current_dict.get(motor_type, 0))
        self.hold_current_percent = hold_current_percent
        self.full_steps = full_steps
        self.micro_steps = micro_steps
        self.motor_type = motor_type

    def __post_init__(self):
        self.write_max_current()
        self.write_hold_current()
        self.write_subdivision()

    def read_encoder_value_carry(self) -> tuple[int, int]:
        """ Read the encoder value """
        encoder_value = self.mb.read_registers(functioncode=4, registeraddress=0x30, number_of_registers=3)
        # The carry is in the first register (16 bits)
        value = encoder_value[2]  # Extract the first register value (assumes unsigned)

        # Combine the next two registers into a single 32-bit signed integer
        msb = encoder_value[0]
        lsb = encoder_value[1]
        carry = (msb << 16) | lsb

        if carry >= 0x80000000:
            carry -= 0x100000000
            value = value - ENCODER_STEPS

        if value >= 0x8000:
            value -= 0x10000

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
        val_mode = mode.value
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
        self.mb.write_register(functioncode=6, registeraddress=0x84, value=self.micro_steps)

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

    def write_no_limit_go_home_parameter(self, max_return_angle: float, no_switch_go_home: bool,
                                         no_limit_current: int) -> None:
        """ Set the no limit go home parameter """
        axis = int(max_return_angle * ANGLE_TO_AXIS)
        axis_low = axis & 0xFFFF
        axis_high = (axis >> 16) & 0xFFFF
        values = [axis_high, axis_low, int(no_switch_go_home), no_limit_current]
        self.mb.write_registers(registeraddress=0x94, values=values)
        time.sleep(0.5)

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
        self.wait_until_motor_status(MotorStatus.STOP)
        time.sleep(1)
        self.wait_until_motor_status(MotorStatus.STOP)


    def emergency_stop(self) -> None:
        """ Stop the motor """
        self.mb.write_register(functioncode=6, registeraddress=0xF7, value=1)

    def move_by_speed(self, direction: MotorDirection, acc: int, speed: int) -> None:
        """ Move the motor at the specified speed """
        self.check_speed(speed)
        self.check_acceleration(acc)

        dir_acc = direction.value << 8 | acc
        values = [dir_acc, speed]
        self.mb.write_registers(registeraddress=0xF6, values=values)

    def save_speed_parameters(self, save_clean: MotorSpeedParameterSaveClean) -> None:
        """ Save or clean the speed parameters """
        self.mb.write_register(functioncode=6, registeraddress=0xFF, value=save_clean.value)

    def move_relative_by_pulses(self, direction: MotorDirection, acc: int, speed: int, pulses: int) -> None:
        """ Move the motor by the specified number of pulses """
        self.check_pulses(pulses)
        self.check_speed(speed)
        self.check_acceleration(acc)
        dir_acc = direction.value << 8 | acc
        pulses_low = pulses & 0xFFFF
        pulses_high = (pulses >> 16) & 0xFFFF
        values = [dir_acc, speed, pulses_high, pulses_low]
        self.mb.write_registers(registeraddress=0xFD, values=values)
        self.wait_until_motor_status(MotorStatus.STOP)

    def move_absolute_by_pulses(self, acc: int, speed: int, pulses: int) -> None:
        """ Move the motor to the specified number of pulses """
        self.check_pulses(pulses)
        self.check_speed(speed)
        self.check_acceleration(acc)
        pulses_low = pulses & 0xFFFF
        pulses_high = (pulses >> 16) & 0xFFFF
        values = [acc, speed, pulses_high, pulses_low]
        self.mb.write_registers(registeraddress=0xFE, values=values)
        self.wait_until_motor_status(MotorStatus.STOP)

    def move_to_relative_axis(self, acc: int, speed: int, axis: int) -> None:
        """ Move the motor by the specified angle """
        self.check_acceleration(acc)
        self.check_speed(speed)
        axis_low = axis & 0xFFFF
        axis_high = (axis >> 16) & 0xFFFF
        values = [acc, speed, axis_high, axis_low]
        self.mb.write_registers(registeraddress=0xF4, values=values)
        self.wait_until_motor_status(MotorStatus.STOP)

    def move_to_absolute_axis(self, acc: int, speed: int, axis: int) -> None:
        """ Move the motor to the specified angle """
        self.check_acceleration(acc)
        self.check_speed(speed)
        axis_low = axis & 0xFFFF
        axis_high = (axis >> 16) & 0xFFFF
        values = [acc, speed, axis_high, axis_low]
        self.mb.write_registers(registeraddress=0xF5, values=values)
        self.wait_until_motor_status(MotorStatus.STOP)

    def move_to_relative_angle(self, acc: int, speed: int, angle: float) -> None:
        """ Move the motor by the specified angle """
        axis = int(angle * ANGLE_TO_AXIS)
        self.move_to_relative_axis(acc, speed, axis)

    def move_to_absolute_angle(self, acc: int, speed: int, angle: float) -> None:
        """ Move the motor to the specified angle """
        axis = int(angle * ANGLE_TO_AXIS)
        self.move_to_absolute_axis(acc, speed, axis)

    def wait_until_motor_status(self, scope_status: MotorStatus) -> None:
        """ Wait until the motor stops """
        while 1:
            status = self.read_motor_status()
            logging.debug("Motor status: %s", status)
            if status == scope_status:
                break

    def check_speed(self, speed: int) -> None:
        """ Check the speed """
        if speed < 0 or speed > 3000:
            raise ValueError("Speed must be between 0 and 3000")

    def check_acceleration(self, acc: int) -> None:
        """ Check the acceleration """
        if acc < 0 or acc > 255:
            raise ValueError("Acceleration must be between 0 and 255")

    def check_pulses(self, pulses: int) -> None:
        """ Check the pulses """
        if pulses < 0 or pulses > 0xFFFFFF:
            raise ValueError("Pulses must be between 0 and 0xFFFFFF")

    def read_angle_carry(self):
        carry, value = self.read_encoder_value_carry()
        angle = carry / ANGLE_TO_AXIS + value / ANGLE_TO_AXIS

        return angle