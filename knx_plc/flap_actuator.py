""" Class for Flap Actuator """
import asyncio
from dataclasses import dataclass

import minimalmodbus


from xknx import XKNX
from xknx.devices import NumericValue, Sensor

from servo import servo

PERCENT_TO_ANGLE_FACTOR = 90/100
FLAP_ANGLE_FACTOR = 320/90


async def update_flap_angle(knx_position: NumericValue, flap_servo: servo.Servo) -> None:
    """Task to update flap angle and send updates to KNX."""
    old_value = 0.0

    while True:
        try:
            await asyncio.sleep(1)
            angle = flap_servo.read_angle_carry()
            percent = 100.0 - float(angle / FLAP_ANGLE_FACTOR / PERCENT_TO_ANGLE_FACTOR)
            percent = max(0.0, min(100.0, percent))

            if percent != old_value:
                old_value = percent
                print(f"Current angle: {angle} Current percent: {percent}")
                await knx_position.set(percent)

        except Exception as e:
            print(f"Error updating flap angle: {e}")
            await asyncio.sleep(5)  # Retry delay after an error


@dataclass
class FlapActuator:
    """Class for Flap Actuator."""
    name: str
    knx_set_point: Sensor
    knx_position: NumericValue
    flap_servo: servo.Servo
    last_position: int
    speed: int
    acc: int
    no_limit_current: int
    angle_factor: int

    def flap_callback(self, knx_sensor: Sensor,) -> None:
        """Run callback when the light changed any of its state."""
        try:
            val = float(knx_sensor.sensor_value.value)
            print(f"{knx_sensor.name} - {val} {knx_sensor.unit_of_measurement()}")
            val = 100 - val
            self.flap_servo.move_to_absolute_angle(speed=self.speed, acc=self.acc,
                                                   angle=val * PERCENT_TO_ANGLE_FACTOR * FLAP_ANGLE_FACTOR)
        except Exception as e:
            print(f"Error in flap_callback: {e}")

    def __init__(self, mb: minimalmodbus.Instrument, xknx: XKNX, name: str,
                 ga_set_point: str, ga_position: str,
                 address: int, max_current: int, hold_current_percent: int, no_limit_current: int,
                 full_steps: int, micro_steps: int,
                 speed: int, acc: int):
        self.speed = speed
        self.acc = acc
        self.flap_servo = servo.Servo(mb, servo.MotorType.SERVO_42_D, address,
                                      max_current, hold_current_percent, full_steps, micro_steps)

        self.flap_servo.write_no_limit_go_home_parameter(max_return_angle=90 * FLAP_ANGLE_FACTOR,
                                                         no_switch_go_home=True,
                                                         no_limit_current=no_limit_current)
        self.flap_servo.go_home()

        self.knx_position = NumericValue(
            xknx=xknx,
            name=name+'_set_point',
            group_address=ga_set_point,
            value_type='percent',
            respond_to_read=True,
        )

        self.knx_set_point = Sensor(
            xknx=xknx,
            name=name + '_position',
            group_address_state=ga_position,
            value_type='percent',
            device_updated_cb=self.flap_callback,
        )

        xknx.devices.async_add(self.knx_position)
        xknx.devices.async_add(self.knx_set_point)


