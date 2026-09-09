from lib.hardware_test_utils import create_hardware


def main() -> None:
    hardware = create_hardware()
    hardware.stop()
    print("Motors stopped.")


if __name__ == "__main__":
    main()
