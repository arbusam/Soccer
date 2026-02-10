"""
Run physical calibration on motor drivers and save results to calibration_data.json.
Run this once (or when motors/encoders change); thereafter use init_motors() which loads the saved file.
"""

from movement import _prompt_i2c_addresses, calibrate_motors, get_motors_for_calibration

if __name__ == "__main__":
    addresses = _prompt_i2c_addresses()
    motors, motor_count, normalized_addresses = get_motors_for_calibration(addresses)
    calibrate_motors(motors, motor_count, normalized_addresses, calibration_file="calibration_data.json")
