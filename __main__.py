""" Example usage of the servo module. """
import time, sys

import minimalmodbus

from servo import scan as scanner, servo


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

test_servo = servo.Servo(instrument, servo.MotorType.SERVO_42_D, 1, 1000, 20, 8)
# test_servo.motor_calibrate()

while 1:
    value = test_servo.read_encoder_value()
    print(f"Encoder value: {value}")
    time.sleep(0.1)
