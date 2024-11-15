import minimalmodbus

import servo.scan as scanner
import servo.servo as servo

# Example usage
available_ports = scanner.list_serial_ports()

if len(available_ports) == 0:
    print("No serial ports found.")
    exit()

print(available_ports)

servo_port = "/dev/cu.usbserial-1420"
print(f"Using serial port: {servo_port}")
found_devices = scanner.scan_modbus(servo_port, start_addr=1, end_addr=10)

if len(found_devices) == 0:
    print("No devices found.")
    exit()

instrument = minimalmodbus.Instrument(servo_port, found_devices[0])
instrument.serial.baudrate = 38400

test_servo = servo.Servo(instrument, servo.MotorType.SERVO_42_D, 1, 1000, 20, 8)
test_servo.motor_calibrate()

