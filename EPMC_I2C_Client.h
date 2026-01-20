// #ifndef EPMC_I2C_H
// #define EPMC_I2C_H

#pragma once

#include <Arduino.h>
#include <Wire.h>

class EPMC_I2C_Client
{
public:
  explicit EPMC_I2C_Client(uint8_t address);
  bool begin(TwoWire &wire=Wire);

  void writeSpeed(float v0, float v1);
  void writePWM(int pwm0, int pwm1);
  void readPos(float &pos0, float &pos1);
  void readVel(float &v0, float &v1);
  void setCmdTimeout(int timeout_ms);
  float getMaxVel(int motor_no);
  int getCmdTimeout();
  void setPidMode(int mode);
  int getPidMode();
  bool clearDataBuffer();
  void readMotorData(float &pos0, float &pos1, float &v0, float &v1);


private:
  TwoWire *_wire = nullptr;
  uint8_t slaveAddr;
  bool wireReady() const;
  uint8_t computeChecksum(const uint8_t *packet, uint8_t length);
  void send_packet_without_payload(uint8_t cmd);
  void write_data1(uint8_t cmd, float val = 0.0f, uint8_t pos = 100);
  void write_data2(uint8_t cmd, float val0, float val1);
  void read_data1(float &val0);
  void read_data2(float &val0, float &val1);
  void read_data4(float &val0, float &val1, float &val2, float &val3);

  // Serial Protocol Command IDs -------------
  static constexpr uint8_t START_BYTE = 0xAA;
  static constexpr uint8_t WRITE_VEL = 0x01;
  static constexpr uint8_t WRITE_PWM = 0x02;
  static constexpr uint8_t READ_POS = 0x03;
  static constexpr uint8_t READ_VEL = 0x04;
  static constexpr uint8_t GET_MAX_VEL = 0x14;
  static constexpr uint8_t SET_PID_MODE = 0x15;
  static constexpr uint8_t GET_PID_MODE = 0x16;
  static constexpr uint8_t SET_CMD_TIMEOUT = 0x17;
  static constexpr uint8_t GET_CMD_TIMEOUT = 0x18;
  static constexpr uint8_t READ_MOTOR_DATA = 0x2A;
  static constexpr uint8_t CLEAR_DATA_BUFFER = 0x2C;
  //---------------------------------------------
};

// #endif