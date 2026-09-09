#include "PowerfulBLDCdriver.h"
#include <cstring>

// Public methods:

bool PowerfulBLDCdriver::begin(uint8_t address, TwoWire* wire) {
  i2c_hardware = wire;
  i2c_address = address;
  QDRformat = 0;
  return true;
}

uint32_t PowerfulBLDCdriver::getFirmwareVersion() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x00);
  i2c_hardware->endTransmission();
  i2c_hardware->requestFrom(i2c_address, (uint8_t)4, (uint8_t)1);
  uint32_t version = *((uint32_t*)receive32bitvalue());
  return version;
}

void PowerfulBLDCdriver::setIqPidConstants(int32_t kp, int32_t ki) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x40);
  send32bitvalue(&kp);
  send32bitvalue(&ki);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setIdPidConstants(int32_t kp, int32_t ki) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x41);
  send32bitvalue(&kp);
  send32bitvalue(&ki);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setSpeedPidConstants(float kp, float ki, float kd) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x42);
  send32bitvalue(&kp);
  send32bitvalue(&ki);
  send32bitvalue(&kd);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setPositionPidConstants(float kp, float ki, float kd) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x43);
  send32bitvalue(&kp);
  send32bitvalue(&ki);
  send32bitvalue(&kd);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setPositionRegionBoundary(float boundary) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x44);
  send32bitvalue(&boundary);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::configureOperatingModeAndSensor(uint8_t operatingmode, uint8_t sensortype) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x20);
  i2c_hardware->write(operatingmode + (sensortype << 4));
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::configureCommandMode(uint8_t commandmode) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x21);
  i2c_hardware->write(commandmode);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setVoltage(int32_t voltage) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x10);
  send32bitvalue(&voltage);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setTorque(int32_t torque) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x11);
  send32bitvalue(&torque);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setSpeed(int32_t speed) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x12);
  send32bitvalue(&speed);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setPosition(uint32_t position, uint8_t elecangle) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x13);
  send32bitvalue(&position);
  send8bitvalue(&elecangle);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setCurrentLimitFOC(int32_t current) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x33);
  send32bitvalue(&current);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setSpeedLimit(int32_t speed) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x34);
  send32bitvalue(&speed);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::clearFaults() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x01);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setELECANGLEOFFSET(uint32_t ELECANGLEOFFSET) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x30);
  send32bitvalue(&ELECANGLEOFFSET);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setEAOPERSPEED(int32_t EAOPERSPEED) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x31);
  send32bitvalue(&EAOPERSPEED);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setSINCOSCENTRE(int32_t SINCOSCENTRE) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x32);
  send32bitvalue(&SINCOSCENTRE);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::setCalibrationOptions(uint32_t voltage, int32_t speed, uint32_t scycles, uint32_t cycles) {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x3A);
  send32bitvalue(&voltage);
  send32bitvalue(&speed);
  send32bitvalue(&scycles);
  send32bitvalue(&cycles);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::startCalibration() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x38);
  i2c_hardware->write(0x01);
  i2c_hardware->endTransmission();
}

void PowerfulBLDCdriver::stopCalibration() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x38);
  i2c_hardware->write(0x00);
  i2c_hardware->endTransmission();
}

bool PowerfulBLDCdriver::isCalibrationFinished() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x39);
  i2c_hardware->endTransmission();
  i2c_hardware->requestFrom(i2c_address, (uint8_t)9, (uint8_t)1);

  uint8_t calibration_state = *((uint8_t*)receive8bitvalue());
  receive32bitvalue();
  receive32bitvalue();

  if (calibration_state == 255) {
    return true;
  }
  return false;
}

uint32_t PowerfulBLDCdriver::getCalibrationELECANGLEOFFSET() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x39);
  i2c_hardware->endTransmission();
  i2c_hardware->requestFrom(i2c_address, (uint8_t)9, (uint8_t)1);

  uint8_t calibration_state = *((uint8_t*)receive8bitvalue());
  uint32_t ELECANGLEOFFSET = *((uint32_t*)receive32bitvalue());
  receive32bitvalue();

  if (calibration_state == 255) {
    return ELECANGLEOFFSET;
  }
  return 0;
}

int32_t PowerfulBLDCdriver::getCalibrationSINCOSCENTRE() {
  i2c_hardware->beginTransmission(i2c_address);
  i2c_hardware->write(0x39);
  i2c_hardware->endTransmission();
  i2c_hardware->requestFrom(i2c_address, (uint8_t)9, (uint8_t)1);

  uint8_t calibration_state = *((uint8_t*)receive8bitvalue());
  receive32bitvalue();
  int32_t SINCOSCENTRE = *((int32_t*)receive32bitvalue());

  if (calibration_state == 255) {
    return SINCOSCENTRE;
  }
  return 0;
}

void PowerfulBLDCdriver::updateQuickDataReadout() {
  if (QDRformat == 0) {
    i2c_hardware->requestFrom(i2c_address, (uint8_t)10, (uint8_t)1);

    QDRposition = *((uint32_t*)receive32bitvalue());
    std::memcpy(&QDRspeed, receive32bitvalue(), sizeof(QDRspeed));
    QDRERROR1 = *((uint8_t*)receive8bitvalue());
    QDRERROR2 = *((uint8_t*)receive8bitvalue());
  }
}

uint32_t PowerfulBLDCdriver::getPositionQDR() { return QDRposition; }
int32_t PowerfulBLDCdriver::getSpeedQDR() { return QDRspeed; }
uint8_t PowerfulBLDCdriver::getERROR1QDR() { return QDRERROR1; }
uint8_t PowerfulBLDCdriver::getERROR2QDR() { return QDRERROR2; }


// *********************************************************************************************************************************************
// Private Methods:

void PowerfulBLDCdriver::send32bitvalue(void* value) {
  uint32_t temp;
  std::memcpy(&temp, value, sizeof(temp));
  i2c_hardware->write(temp & 0xFF);
  i2c_hardware->write((temp >> 8) & 0xFF);
  i2c_hardware->write((temp >> 16) & 0xFF);
  i2c_hardware->write((temp >> 24) & 0xFF);
}

void PowerfulBLDCdriver::send16bitvalue(void* value) {
  uint16_t temp;
  std::memcpy(&temp, value, sizeof(temp));
  i2c_hardware->write(temp & 0xFF);
  i2c_hardware->write((temp >> 8) & 0xFF);
}

void PowerfulBLDCdriver::send8bitvalue(void* value) {
  uint8_t temp = *(uint8_t*)value;
  i2c_hardware->write(temp & 0xFF);
}

void* PowerfulBLDCdriver::receive32bitvalue() {
  static thread_local uint32_t value;
  value = 0;
  value = i2c_hardware->read();
  value += static_cast<uint32_t>(i2c_hardware->read()) << 8;
  value += static_cast<uint32_t>(i2c_hardware->read()) << 16;
  value += static_cast<uint32_t>(i2c_hardware->read()) << 24;
  return &value;
}

void* PowerfulBLDCdriver::receive16bitvalue() {
  static thread_local uint16_t value;
  value = 0;
  value = i2c_hardware->read();
  value += static_cast<uint32_t>(i2c_hardware->read()) << 8;
  return &value;
}

void* PowerfulBLDCdriver::receive8bitvalue() {
  static thread_local uint8_t value;
  value = 0;
  value = i2c_hardware->read();
  return &value;
}


