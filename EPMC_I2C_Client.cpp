#include "EPMC_I2C_Client.h"

inline void delayMs(int ms)
{
  for (int i = 0; i < ms; i += 1)
  {
    delayMicroseconds(1000);
  }
}

EPMC_I2C_Client::EPMC_I2C_Client(uint8_t slave_addr, SupportedNumOfMotors supported_num_of_motors)
{
  slaveAddr = slave_addr;

  switch (supported_num_of_motors)
  {
  case SupportedNumOfMotors::TWO :
    num_of_motors = 2;
    break;
  
  case SupportedNumOfMotors::FOUR :
    num_of_motors = 4;
    break;
  }
}

bool EPMC_I2C_Client::begin(TwoWire &wire)
{
  _wire = &wire;

  delayMs(3000);

  constexpr int max_attempts = 10;

  for (int i = 0; i < max_attempts; ++i)
  {
      if (confirmNumOfMotors()){
        return true;
      }
      delayMs(100);
  }

  _wire = nullptr;
  return false;
}

bool EPMC_I2C_Client::wireReady() const {
  return _wire != nullptr;
}

uint8_t EPMC_I2C_Client::computeChecksum(const uint8_t *packet, uint8_t length){
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += packet[i]; 
  }
  return sum & 0xFF; 
}

void EPMC_I2C_Client::send_packet_without_payload(uint8_t cmd)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + checksum
  uint8_t packet[4];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 0; // msg length = 0

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 3);
  packet[3] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EPMC_I2C_Client::write_data1(uint8_t cmd, float val, uint8_t pos)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + pos + float + checksum
  uint8_t packet[1 + 1 + 1 + 1 + 4 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 5; // msg is uint8 + float = 5byte length
  packet[3] = pos;
  memcpy(&packet[4], &val, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 8);
  packet[8] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EPMC_I2C_Client::write_data2(uint8_t cmd, float val0, float val1)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + float*4 + checksum
  uint8_t packet[1 + 1 + 1 + 8 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 8; // msg is 2 float = 8byte length
  memcpy(&packet[3], &val0, sizeof(float));
  memcpy(&packet[7], &val1, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 11);
  packet[11] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EPMC_I2C_Client::write_data4(uint8_t cmd, float val0, float val1, float val2, float val3)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + float*4 + checksum
  uint8_t packet[1 + 1 + 1 + 16 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 16; // msg is 4 float = 16byte length
  memcpy(&packet[3], &val0, sizeof(float));
  memcpy(&packet[7], &val1, sizeof(float));
  memcpy(&packet[11], &val2, sizeof(float));
  memcpy(&packet[15], &val3, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 19);
  packet[19] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EPMC_I2C_Client::read_data1(float& val0)
{
  if (!wireReady()) return;
  uint8_t buffer[4];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)4);
  if (dataSizeInBytes != 4) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
}

void EPMC_I2C_Client::read_data2(float &val0, float &val1)
{
  if (!wireReady()) return;
  uint8_t buffer[8];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)8);
  if (dataSizeInBytes != 8) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
}

void EPMC_I2C_Client::read_data4(float &val0, float &val1, float &val2, float &val3)
{
  if (!wireReady()) return;
  uint8_t buffer[16];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)16);
  if (dataSizeInBytes != 16) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
  memcpy(&val2, &buffer[8], sizeof(float));
  memcpy(&val3, &buffer[12], sizeof(float));
}

void EPMC_I2C_Client::writeSpeed(float v0, float v1){
  if (num_of_motors != 2) return; 
  write_data2(WRITE_VEL, v0, v1);
}

void EPMC_I2C_Client::writeSpeed(float v0, float v1, float v2, float v3){
  if (num_of_motors != 4) return;
  write_data4(WRITE_VEL, v0, v1, v2, v3);
}

void EPMC_I2C_Client::writePWM(int pwm0, int pwm1){
  if (num_of_motors != 2) return;
  write_data2(WRITE_PWM, (float)pwm0, (float)pwm1);
}

void EPMC_I2C_Client::writePWM(int pwm0, int pwm1, int pwm2, int pwm3){
  if (num_of_motors != 4) return;
  write_data4(WRITE_PWM, (float)pwm0, (float)pwm1, (float)pwm2, (float)pwm3);
}

void EPMC_I2C_Client::readPos(float &pos0, float &pos1){
  if (num_of_motors != 2) return;
  send_packet_without_payload(READ_POS);
  read_data2(pos0, pos1);
}

void EPMC_I2C_Client::readPos(float &pos0, float &pos1, float &pos2, float &pos3){
  if (num_of_motors != 4) return;
  send_packet_without_payload(READ_POS);
  read_data4(pos0, pos1, pos2, pos3);
  read_data4(pos0, pos1, pos2, pos3);
}

void EPMC_I2C_Client::readVel(float &v0, float &v1){
  if (num_of_motors != 2) return;
  send_packet_without_payload(READ_VEL);
  read_data2(v0, v1);
}

void EPMC_I2C_Client::readVel(float &v0, float &v1, float &v2, float &v3){
  if (num_of_motors != 4) return;
  send_packet_without_payload(READ_VEL);
  read_data4(v0, v1, v2, v3);
  read_data4(v0, v1, v2, v3);
}

float EPMC_I2C_Client::getMaxVel(int motor_no){
  float max_vel=0.0;
  write_data1(GET_MAX_VEL, 0.0, motor_no);
  if (num_of_motors == 2) {
    read_data1(max_vel);
  }
  else if (num_of_motors == 4) {
    read_data1(max_vel);
    read_data1(max_vel);
  }
  return max_vel;
}

void EPMC_I2C_Client::setCmdTimeout(int timeout_ms){
  write_data1(SET_CMD_TIMEOUT, (float)timeout_ms);
}

int EPMC_I2C_Client::getCmdTimeout(){
  float timeout_ms=0.0;
  write_data1(GET_CMD_TIMEOUT);
  if (num_of_motors == 2) {
    read_data1(timeout_ms);
  }
  else if (num_of_motors == 4) {
    read_data1(timeout_ms);
    read_data1(timeout_ms);
  }
  return (int)timeout_ms;
}

void EPMC_I2C_Client::setPidMode(int mode){
  write_data1(SET_PID_MODE, (float)mode);
}

int EPMC_I2C_Client::getPidMode(){
  float mode=0.0;
  write_data1(GET_PID_MODE);
  if (num_of_motors == 2) {
    read_data1(mode);
  }
  else if (num_of_motors == 4) {
    read_data1(mode);
    read_data1(mode);
  }
  return (int)mode;
}

bool EPMC_I2C_Client::clearDataBuffer(){
  float res=0.0;
  write_data1(CLEAR_DATA_BUFFER);
  if (num_of_motors == 2) {
    read_data1(res);
  }
  else if (num_of_motors == 4) {
    read_data1(res);
    read_data1(res);
  }
  return ((int)res == 1);
}

int EPMC_I2C_Client::getNumOfMotors(){
  float num_of_mtr=0.0;
  write_data1(GET_NUM_OF_MOTORS);
  if (num_of_motors == 2) {
    read_data1(num_of_mtr);
  }
  else if (num_of_motors == 4) {
    read_data1(num_of_mtr);
    read_data1(num_of_mtr);
  }
  return (int)num_of_mtr;
}

bool EPMC_I2C_Client::confirmNumOfMotors()
{
  int motor_num = getNumOfMotors();
  Serial.println(motor_num);
  return (motor_num == num_of_motors);
}