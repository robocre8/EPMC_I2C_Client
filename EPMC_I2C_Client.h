// #ifndef EPMC_I2C_H
// #define EPMC_I2C_H

#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class SupportedNumOfMotors: int {
  TWO = 2,
  FOUR = 4
};

class EPMC_I2C_Client
{
public:
  float dataBuffer[4];
  explicit EPMC_I2C_Client(uint8_t address, SupportedNumOfMotors supported_num_of_motors);
  bool begin(TwoWire &wire=Wire);
  bool confirmNumOfMotors();
  int getNumOfMotors();

  void writeSpeed(float v0, float v1, float v2=0.0, float v3=0.0);
  void writePWM(int pwm0, int pwm1, int pwm2=0, int pwm3=0);
  void readPos();
  void readVel();
  void setCmdTimeout(int timeout_ms);
  float getMaxVel(int motor_no);
  int getCmdTimeout();
  void setPidMode(int mode);
  int getPidMode();
  bool clearDataBuffer();


private:
  TwoWire *_wire = nullptr;
  uint8_t slaveAddr;
  int num_of_motors = 0;
  bool wireReady() const;
  uint8_t computeChecksum(const uint8_t *packet, uint8_t length);
  void send_packet_without_payload(uint8_t cmd);
  void write_data1(uint8_t cmd, float val = 0.0f, uint8_t pos = 100);
  void write_data2(uint8_t cmd, float val0, float val1);
  void write_data4(uint8_t cmd, float val0, float val1, float val2, float val3);
  void read_data1(float &val0);
  void read_data2(float &val0, float &val1);
  void read_data4(float &val0, float &val1, float &val2, float &val3);
  void clearBuffer();

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
  static constexpr uint8_t GET_NUM_OF_MOTORS = 0x2D;
  //---------------------------------------------
};

// #endif