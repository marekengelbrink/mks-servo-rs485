""" Example usage of the servo module. """
import time, sys, logging
import minimalmodbus
from servo import scan as scanner, servo
from knx_plc import flap_actuator as flap_actuator
import asyncio

from xknx import XKNX
from xknx.io import ConnectionConfig, ConnectionType

FLAP_ANGLE_FACTOR = 320/90

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


async def main():
    """Main function."""
    xknx = XKNX(connection_config=ConnectionConfig(connection_type=ConnectionType.AUTOMATIC))

    try:
        await xknx.start()

        test_flap = flap_actuator.FlapActuator(
            name="Flap Actuator",
            xknx=xknx,
            ga_position="30/7/41",
            ga_set_point="30/7/40",
            mb=instrument,
            address=1,
            max_current=2000,
            hold_current_percent=20,
            no_limit_current=1000,
            full_steps=200,
            micro_steps=16,
            speed=20,
            acc=1,
        )

        update_task = asyncio.create_task(flap_actuator.update_flap_angle(test_flap.knx_position,
                                                                          test_flap.flap_servo))
        await update_task

    except asyncio.CancelledError:
        print("Program cancelled, stopping gracefully.")

    finally:
        await xknx.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exiting...")
