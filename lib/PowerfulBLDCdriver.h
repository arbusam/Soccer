/*!
 * @file PowerfulBLDCdriver.h
 * 
 * @mainpage Library for easy communication with the Powerful Brushless Motor Driver.
 * @section intro_sec Introduction
 * Library for easy communication with the Powerful Brushless Motor Driver.
 * @section dependencies Dependencies
 * Wire.h
 * @section author Author
 * Board design, firmware, and library written by Andrew
 * @section license License
 * This work is marked with CC0 1.0 Universal. To view a copy of this license, visit https://creativecommons.org/publicdomain/zero/1.0/
 */

#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#else
#include "linux_wire.h"
#endif

class PowerfulBLDCdriver {
public:
  /*!
   * @brief Initialises the communication settings.
   * @param address The I2C address of the motor
   * @param wire The I2C interface to use, defaults to Wire
   * @return True on success and False on failure
   */
  bool begin(uint8_t address, TwoWire* wire);

  /*!
   * @brief Gets the firmware version of the motor driver. 
   * @return The version number
   */
  uint32_t getFirmwareVersion();

  /*!
   * @brief Set the Quick Data Readout format. See the datasheet for details.
   * @param format Format Byte. See the datasheet for details. 
   */
  void setQuickDataReadoutFormat(uint8_t format);

  /*!
   * @brief Requests the Quick Data Readout from the motor driver and stores it in a buffer. Call this function to update buffer with fresh data. Call other functions to get the data itself.
   */
  void getQuickDataReadout();

  /*!
   * @brief Gets the motor position from Quick Data Readout. 
   * @return 32bit position, where 1LSB = 1 electrical revolution.
   */
  uint32_t getPosition();

  /*!
   * @brief Sets the PID kp and ki constants for the Q_current controller in FOC.
   * @param kp The kp constant. Range between 0 and 1024.
   * @param ki The ki constant. Range between 0 and 1024.
   */
  void setIqPidConstants(int32_t kp, int32_t ki);

  /*!
   * @brief Sets the PID kp and ki constants for the D_current controller in FOC.
   * @param kp The kp constant. Range between 0 and 1024.
   * @param ki The ki constant. Range between 0 and 1024.
   */
  void setIdPidConstants(int32_t kp, int32_t ki);

  /*!
   * @brief Sets the PID kp and ki constants for the speed controller. Note: The constants are not interchangeable between Trapezoidal, Sinusoidal, and FOC operating modes.
   * @param kp The kp constant. No hard range limit, but suggest starting small.
   * @param ki The ki constant. No hard range limit, but suggest starting small.
   * @param kd The kd constant. No hard range limit, but suggest starting small.
   */
  void setSpeedPidConstants(float kp, float ki, float kd);

  /*!
   * @brief Sets the PID kp and ki constants for the position controller. Note: The constants are not interchangeable between Trapezoidal, Sinusoidal, and FOC operating modes.
   * @param kp The kp constant. No hard range limit, but suggest starting small.
   * @param ki The ki constant. No hard range limit, but suggest starting small.
   * @param kd The kd constant. No hard range limit, but suggest starting small.
   */
  void setPositionPidConstants(float kp, float ki, float kd);

  /*!
   * @brief Change boundary between linear and sqrt for the position PID controller. Boundary position proportional to pos_region_boundary^2.
   * @param boundary The boundary. 65536 = 1 POS. Set to 4294967296 if constant acceleration model position control is not desired, linear will be used instead. PID constants will change.
   */
  void setPositionRegionBoundary(float boundary);

  /*!
   * @brief Sets the operating mode and sensor type. Motor must be stopped before sending this command.
   * @param operatingmode 1 = Trapezoidal, 2 = Sinusoidal, 3 = Field Oriented Control (FOC), 15 = Calibration Mode
   * @param sensortype 1 = Sin/Cos Encoder, 2 = Incremental Encoder, 3 = Sensorless
   */
  void configureOperatingModeAndSensor(uint8_t operatingmode, uint8_t sensortype);

  /*!
   * @brief Sets the command mode. Motor must be stopped before sending this command.
   * @param commandmode 1 = Voltage (Trapezoidal or Sinusoidal), 2 = Torque (FOC), 12 = Speed (All Operating Modes), 13 = Position (All Operating Modes) 15 = Calibration Mode
   */
  void configureCommandMode(uint8_t commandmode);

  /*!
   * @brief Sends a voltage command.
   * @param voltage Between 0 and 3399, where 0 = 0V and 3399 = input voltage. Values under 250 do not work well.
   */
  void setVoltage(int32_t voltage);

  /*!
   * @brief Sends a torque command.
   * @param torque 1LSB = 2^-16 Amps of Q_current which is proportional to torque. See motor datasheet for the relationship.
   */
  void setTorque(int32_t torque);

  /*!
   * @brief Sends a speed command.
   * @param speed 1LSB = 2^-16 electrical revolutions per second.
   */
  void setSpeed(int32_t speed);

  /*!
   * @brief Sends a position command.
   * @param position 1LSB = 1 electrical revolution.
   * @param elecangle 1LSB = 1/256 electrical revolutions.
   */
  void setPosition(uint32_t position, uint8_t elecangle);

  /*!
   * @brief Sets the output peak current limit. Only works in FOC operating mode.
   * @param current 1LSB = 2^-16 Amps. Maximum value is 524288 which is 8 Amps.
   */
  void setCurrentLimitFOC(int32_t current);

  /*!
   * @brief Sets the speed limit. Only works in Speed or Position mode.
   * @param speed 1LSB = 2^-16 electrical revolutions per second. Range between 0 and 546133333.
   */
  void setSpeedLimit(int32_t speed);

  /*!
   * @brief Clears any latched faults on the motor driver.
   */
  void clearFaults();

  /*!
   * @brief Sets the ELECANGLEOFFSET calibration parameter. This is used to align the motor encoder with the electrical angle of the motor.
   * @param ELECANGLEOFFSET 2^32 = 1 electrical revolution
   */
  void setELECANGLEOFFSET(uint32_t ELECANGLEOFFSET);

  /*!
   * @brief Sets the EAOPERSPEED calibration parameter for Sinusoidal control. This is used to compensate the phase shift between voltage and current at high speeds.
   * @param EAOPERSPEED 1LSB = 1 ELECANGLEOFFSET/(POS/second)
   */
  void setEAOPERSPEED(int32_t EAOPERSPEED);

  /*!
   * @brief Sets the SINCOSCENTRE calibration parameter for Sin/Cos Encoders. This is the center voltage of the Sin/Cos encoder.
   * @param SINCOSCENTRE 0 = 0V, 4096 = 3.3V. Valid range is between 0 and 4096.
   */
  void setSINCOSCENTRE(int32_t SINCOSCENTRE);

  /*!
   * @brief Sets the options used during calibration.
   * @param voltage 0 = 0V, 3399 = supply voltage. Recommended starting value: 250. If motor does not spin try increasing this slowly. Beware of motor overcurrent.
   * @param speed 1LSB = 2^-16 electrical revolutions per second. Recommended starting value: 1048576. If motor does not spin try lowering this and increasing calibration time.
   * @param scycles Settling time in PWM cycles. 50000 = 1 second. Recommended starting value: 50000.
   * @param cycles Calibration time in PWM cycles. 50000 = 1 second. Recommended starting value: 500000.
   */
  void setCalibrationOptions(uint32_t voltage, int32_t speed, uint32_t scycles, uint32_t cycles);

  /*!
   * @brief Starts calibration of the motor. Please enter calibration mode and set the calibration options before starting calibration.
   */
  void startCalibration();

  /*!
   * @brief Stop calibration of the motor.
   */
  void stopCalibration();

  /*!
   * @brief Stop calibration of the motor.
   * @return true if calibration is finished and false otherwise.
   */
  bool isCalibrationFinished();

  /*!
   * @brief Get ELECANGLEOFFSET after calibration is finished. You can save the calibration value and send it to the motor driver on startup, only if you are using sin/cos encoder.
   * @return ELECANGLEOFFSET
   */
  uint32_t getCalibrationELECANGLEOFFSET();

  /*!
   * @brief Get SINCOSCENTRE after calibration is finished. Only relevant for sin/cos encoder. You can save the calibration value and send it to the motor driver on startup.
   * @return SINCOSCENTRE
   */
  int32_t getCalibrationSINCOSCENTRE();

  /*!
   * @brief Update the Quick Data Readout cache. Call this function just before reading values using the Quick Data Readout.
   */
  void updateQuickDataReadout();

  /*!
   * @brief Gets the position from the Quick Data Readout cache.
   * @return Position, 1LSB = 1 electrical revolution
   */
  uint32_t getPositionQDR();

  /*!
   * @brief Gets the speed from the Quick Data Readout cache.
   * @return Speed, 1LSB = 2^-16 POS/second
   */
  int32_t getSpeedQDR();

  /*!
   * @brief Gets the ERROR1 byte from the Quick Data Readout cache.
   * @return ERROR1. See datasheet for documentation.
   */
  uint8_t getERROR1QDR();

  /*!
   * @brief Gets the ERROR2 byte from the Quick Data Readout cache.
   * @return ERROR2. See datasheet for documentation.
   */
  uint8_t getERROR2QDR();

private:
  void send32bitvalue(void* value);
  void send16bitvalue(void* value);
  void send8bitvalue(void* value);
  void* receive32bitvalue();
  void* receive16bitvalue();
  void* receive8bitvalue();

  TwoWire* i2c_hardware;
  uint8_t i2c_address;
  uint8_t QDRformat;
  uint32_t QDRposition;
  int32_t QDRspeed;
  uint8_t QDRERROR1;
  uint8_t QDRERROR2;
};