"""
I2C Address Scanner
Scans the I2C bus and prints all connected device addresses.
"""

import sys

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        print("Error: smbus2 or smbus library not found.")
        print("Install with: pip install smbus2")
        sys.exit(1)


def scan_i2c_bus(bus_number=1):
    """
    Scan the I2C bus for connected devices.
    
    Args:
        bus_number: I2C bus number (default 1 for Raspberry Pi)
    
    Returns:
        List of detected I2C addresses
    """
    detected_addresses = []
    
    try:
        bus = smbus.SMBus(bus_number)
    except FileNotFoundError:
        print(f"Error: I2C bus {bus_number} not found.")
        print("Make sure I2C is enabled on your system.")
        return detected_addresses
    except PermissionError:
        print(f"Error: Permission denied accessing I2C bus {bus_number}.")
        print("Try running with sudo or add user to i2c group.")
        return detected_addresses
    
    print(f"Scanning I2C bus {bus_number}...")
    print("-" * 40)
    
    # I2C addresses range from 0x03 to 0x77
    # 0x00-0x02 and 0x78-0x7F are reserved
    for address in range(0x03, 0x78):
        try:
            # Try to read a byte from the device
            bus.read_byte(address)
            detected_addresses.append(address)
        except OSError:
            # No device at this address
            pass
    
    bus.close()
    return detected_addresses


def print_address_grid(detected_addresses):
    """Print I2C addresses in a grid format similar to i2cdetect."""
    print("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")
    
    for row in range(8):
        row_str = f"{row:02x}: "
        for col in range(16):
            address = row * 16 + col
            if address < 0x03 or address > 0x77:
                row_str += "   "
            elif address in detected_addresses:
                row_str += f"{address:02x} "
            else:
                row_str += "-- "
        print(row_str)


def main():
    # Default to bus 1 (common for Raspberry Pi)
    bus_number = 1
    
    if len(sys.argv) > 1:
        try:
            bus_number = int(sys.argv[1])
        except ValueError:
            print(f"Invalid bus number: {sys.argv[1]}")
            print("Usage: python i2c_scan.py [bus_number]")
            sys.exit(1)
    
    detected = scan_i2c_bus(bus_number)
    
    if detected:
        print(f"\nFound {len(detected)} device(s):")
        for addr in detected:
            print(f"  0x{addr:02x} ({addr})")
        
        print_address_grid(detected)
    else:
        print("\nNo I2C devices found.")
    
    print()


if __name__ == "__main__":
    main()

