""" Example usage of the servo module. """
import time, sys, logging
import minimalmodbus
from servo import scan as scanner, servo
import asyncio

from xknx import XKNX
from xknx.devices import NumericValue, RawValue, Sensor
from xknx.io import ConnectionConfig, ConnectionType


FLAP_ANGLE_FACTOR = 320/90
PERCENT_TO_ANGLE_FACTOR = 90/100


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


# Example usage
available_ports = scanner.list_serial_ports()

if len(available_ports) == 0:
    print("No serial ports found.")
    sys.exit()

print(available_ports)

SERVO_PORT = "/dev/cu.usbserial-1420"
print(f"Using serial port: {SERVO_PORT}")
found_devices = scanner.scan_modbus(SERVO_PORT, start_addr=1, end_addr=1)

if len(found_devices) == 0:
    print("No devices found.")
    sys.exit()

instrument = minimalmodbus.Instrument(SERVO_PORT, found_devices[0])
instrument.serial.baudrate = 38400

test_servo = servo.Servo(instrument, servo.MotorType.SERVO_42_D, 1,
                         2000, 20, 200, 16)

time.sleep(1)

print(test_servo.read_motor_status())
'''
test_servo.write_no_limit_go_home_parameter(max_return_angle=90*FLAP_ANGLE_FACTOR, no_switch_go_home=True,
                                            no_limit_current=1000)

test_servo.go_home()

test_servo.move_to_absolute_angle(speed=20, acc=1, angle=50*PERCENT_TO_ANGLE_FACTOR*FLAP_ANGLE_FACTOR)
'''


def flap_callback(knx_sensor: Sensor) -> None:
    """Run callback when the light changed any of its state."""
    try:
        val = float(knx_sensor.sensor_value.value)
        print(f"{knx_sensor.name} - {val} {knx_sensor.unit_of_measurement()}")
        val = 100 - val
        test_servo.move_to_absolute_angle(speed=20, acc=1, angle=val * PERCENT_TO_ANGLE_FACTOR * FLAP_ANGLE_FACTOR)
    except Exception as e:
        print(f"Error in flap_callback: {e}")


async def main():
    xknx = XKNX(connection_config=ConnectionConfig(connection_type=ConnectionType.AUTOMATIC))

    try:
        await xknx.start()

        is_value = NumericValue(
            xknx=xknx,
            name='Flap Angle',
            group_address='30/7/40',
            value_type='percent',
            respond_to_read=True,
        )

        set_point_angle = Sensor(
            xknx=xknx,
            name='Flap Angle Setpoint',
            group_address_state='30/7/41',
            value_type='percent',
            device_updated_cb=flap_callback,
        )

        xknx.devices.async_add(is_value)
        xknx.devices.async_add(set_point_angle)

        old_value = 0
        while 1:
            await asyncio.sleep(1)
            angle = test_servo.read_angle_carry()
            percent = 100.0 - int(angle / FLAP_ANGLE_FACTOR / PERCENT_TO_ANGLE_FACTOR)
            if percent == old_value:
                continue
            old_value = percent
            if percent < 0:
                percent = 0
            if percent > 100:
                percent = 100
            print(f"Current angle: {angle} Current percent: {percent}")
            await is_value.set(percent)

    finally:
        await xknx.stop()




if __name__ == "__main__":
    asyncio.run(main())




