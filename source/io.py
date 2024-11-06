import time
from enum import Enum
import serial
import struct
from crc import Calculator, Crc16


class Status(Enum):
    IN_1 = 0
    IN_2 = 1
    OUT_1 = 2
    OUT_2 = 3


def read_io(ser: serial.Serial, address: int, io: int) -> int:
    """ Read the value of the specified IO from the servo over RS485 """

    # Build command without CRC
    command = bytes([address, 0x04, 0x00, 0x34, 0x00, 0x01])

    # Calculate CRC for the command
    crc_calculator = Calculator(Crc16.MODBUS)
    crc = crc_calculator.checksum(command)
    command += struct.pack("<H", crc)  # Append calculated CRC

    # Send command
    ser.write(command)
    time.sleep(0.1)  # Delay for response

    # Read the response (expected 8 bytes)
    response = ser.read(8)
    if len(response) != 8:
        raise ValueError("Incomplete response received from the device")

    # Parse the response
    received_address = response[0]
    function = response[1]
    byte_count = response[2]
    reserved = response[3]
    status_byte = response[4]

    # Extract boolean values from the status byte's first 4 bits
    bool_values = [(status_byte >> i) & 1 for i in range(3, -1, -1)]

    # Extract and verify CRC16 from the response
    received_crc = struct.unpack("<H", response[-2:])[0]
    calc_crc = crc_calculator.checksum(response[:-2])

    # Verify CRC validity
    if calc_crc != received_crc:
        raise ValueError("CRC check failed for the response")

    # Return the requested IO value
    return bool_values[io]


# Example usage
# ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
# io_value = read_io(ser, address=1, io=Status.IN_1.value)
# print(f"IO value: {io_value}")

def write_io(ser: serial.Serial, out1: int, out2: int, out1_mask: int = 1, out2_mask: int = 1) -> None:
    """ Write the specified values to OUT1 and OUT2 IO ports with masks """

    # Validate input
    if out1 not in [0, 1] or out2 not in [0, 1]:
        raise ValueError("OUT1 and OUT2 values must be 0 or 1.")
    if out1_mask not in [0, 1, 2] or out2_mask not in [0, 1, 2]:
        raise ValueError("OUT1_mask and OUT2_mask values must be 0, 1, or 2.")

    # Build the command without CRC
    command = [
        0x01,  # Slave address
        0x10,  # Function code
        0x00, 0x36,  # Starting address Hi, Lo
        0x00, 0x02,  # Quantity of registers Hi, Lo
        0x04,  # Byte count
        out2_mask,  # OUT2_mask
        out2,  # OUT2 value
        out1_mask,  # OUT1_mask
        out1  # OUT1 value
    ]
    command_bytes = bytes(command)

    # Calculate and append CRC16
    crc_calculator = Calculator(Crc16.MODBUS)
    crc = crc_calculator.checksum(command_bytes)
    command_bytes += struct.pack("<H", crc)

    # Send command over serial
    ser.write(command_bytes)

    # Read response (expected 8 bytes as per image)
    response = ser.read(8)
    if len(response) != 8:
        raise ValueError("Incomplete response received from the device")

    # Parse response to validate it
    received_crc = struct.unpack("<H", response[-2:])[0]
    calc_crc = crc_calculator.checksum(response[:-2])

    if calc_crc != received_crc:
        raise ValueError("CRC check failed for the response")

    print("Write operation successful, CRC is valid.")

# Example usage:
# ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
# write_io(ser, out1=1, out2=0, out1_mask=1, out2_mask=1)