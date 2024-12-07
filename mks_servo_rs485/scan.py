''' Scanner Interface for devices '''
import minimalmodbus
import serial
import serial.tools.list_ports


def list_serial_ports():
    ''' List serial ports '''
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def scan_modbus(port, baudrate=38400, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.5, start_addr=1, end_addr=254) -> list:
    ''' List modbus devices '''
    found_devices = []

    # Set up the instrument with a dummy address (it will change in the loop)
    instrument = minimalmodbus.Instrument(port, 1)  # Port name, and dummy slave address
    instrument.serial.baudrate = baudrate
    instrument.serial.timeout = timeout
    instrument.serial.parity = parity
    instrument.serial.stopbits = stopbits
    instrument.mode = minimalmodbus.MODE_RTU

    print("Scanning for Modbus devices...")

    for address in range(start_addr, end_addr + 1):
        instrument.address = address
        try:
            # Try to read a register to test if the device responds
            # Register address and number of decimals
            instrument.read_registers(functioncode=4, registeraddress=0x30, number_of_registers=3)

            print(f"Device found at address {address}")
            found_devices.append(address)
        except (minimalmodbus.NoResponseError, minimalmodbus.InvalidResponseError):
            # No response or invalid response - skip this address
            print("No response from address:", address)

    if found_devices:
        print(f"Found devices at addresses: {found_devices}")
    else:
        print("No devices found.")

    return found_devices
