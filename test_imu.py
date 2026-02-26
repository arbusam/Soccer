#!/usr/bin/env python3
"""
Simple I2C probe/reader.

Example:
    python test_imu.py --address 0x68 --bus 1 --length 16 --interval 0.2
"""

import argparse
import time

from smbus2 import SMBus, i2c_msg


def parse_address(value: str) -> int:
    """Accept decimal or hex addresses (e.g. 104 or 0x68)."""
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read raw bytes from an I2C device and print what is received."
    )
    parser.add_argument(
        "--bus",
        type=int,
        default=1,
        help="I2C bus number (default: 1)",
    )
    parser.add_argument(
        "--address",
        type=parse_address,
        required=True,
        help="I2C device address, e.g. 0x68",
    )
    parser.add_argument(
        "--length",
        type=int,
        default=16,
        help="Number of bytes to read each cycle (default: 16)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Delay between reads in seconds (default: 0.2)",
    )
    args = parser.parse_args()

    print(
        f"Reading from I2C bus {args.bus}, address 0x{args.address:02X}, "
        f"{args.length} byte(s) every {args.interval}s"
    )
    print("Press Ctrl+C to stop.\n")

    with SMBus(args.bus) as bus:
        while True:
            try:
                read = i2c_msg.read(args.address, args.length)
                bus.i2c_rdwr(read)
                data = list(read)
                hex_data = " ".join(f"{b:02X}" for b in data)
                print(f"RX ({len(data)} bytes): [{hex_data}]  {data}")
            except OSError as exc:
                print(f"I2C read error at 0x{args.address:02X}: {exc}")
            time.sleep(args.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
