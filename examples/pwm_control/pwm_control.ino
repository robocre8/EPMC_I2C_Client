#include <EPMC_I2C_Client.h>

uint8_t i2c_address = 0x55; // set this address to the same address you have during setup via the GUI app
EPMC_I2C_Client controller(i2c_address);
// OR
// EPMC_I2C_Client controller(i2c_address, SupportedNumOfMotors::TWO);

void setup()
{
  // setup serial communication to print result on serial minitor
  Serial.begin(115200);

  // start i2c communication
  Wire.begin();
  bool connect_success = controller.begin(); // takes about 3 to 4 secs to connect
  if (!connect_success) {
    Serial.println("Error Connecting to EPMC. Probably due to Supported NumOfMotor Mismatch");
    while(true);
  }

  controller.clearDataBuffer();
  controller.writePWM(0, 0);

  int cmd_vel_timeout = 10000; // 0 to deactivate.
  controller.setCmdTimeout(cmd_vel_timeout); // set motor command velocity timeout
  cmd_vel_timeout = controller.getCmdTimeout(); // get the stored command velocity timeout
  Serial.print("motor command vel timeout in ms: ");
  Serial.println(cmd_vel_timeout);

}

void loop()
{
  delay(3000);
  controller.writePWM(100, 100);
  delay(5000);
  controller.writePWM(0, 0);

  // float pos0, pos1, vel0, vel1;
  // controller.readPos(pos0, pos1);
  // controller.readVel(vel0, vel1);
  // Serial.print(pos0);
}