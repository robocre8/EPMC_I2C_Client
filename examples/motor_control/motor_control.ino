/*
 * Basic example code on how to control via I2C your geared DC motor with quadrature
 * encoder which is already connected to the Easy PID Motor Controller module and have already
 * succesfully set up their velocity PID control using the epmc_setup_application
 *
 * The code basically sends a low target velocity (in rad/s), waits for some time and then
 * sends a high target velocity (in rad/s). it also prints out the motors' angular positions
 * (in rad) and angular velocities (in rad/s).
 *
 * you can copy the code and use it in your project as you will.
 */

// Easy PID Motor Control i2c communication library
#include <EPMC_I2C_Client.h>

uint8_t i2c_address = 0x55; // set this address to the same address you have during setup via the GUI app
EPMC_I2C_Client controller(i2c_address);

///////// my sepcial delay function ///////////////
void delayMs(int ms)
{
  for (int i = 0; i < ms; i += 1)
  {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////

// [4 rev/sec, 2 rev/sec, 1 rev/sec, 0.5 rev/sec]
float targetVel[] = {1.571, 3.142, 6.284, 12.568}; // in rad/sec
float vel = targetVel[2]; // in rad/sec
float v = 0.0;

float pos0, pos1;
float vel0, vel1;

bool sendHigh = true;

long readTime;
long readTimeInterval = 20; // millisec (50Hz)

long cmdTime;
long cmdTimeInterval = 5000; // millisec

void setup()
{
  // setup serial communication to print result on serial minitor
  Serial.begin(115200);

  // start i2c communication
  Wire.begin();
  controller.begin();

  for (int i=0; i<4; i+=1){
    delay(1000);
    Serial.print("waiting for EPMC controller: ");
    Serial.print(i+1);
    Serial.println(" sec");
  }

  controller.clearDataBuffer();
  controller.writeSpeed(v, v);

  int cmd_vel_timeout = 10000; // 0 to deactivate.
  controller.setCmdTimeout(cmd_vel_timeout); // set motor command velocity timeout
  cmd_vel_timeout = controller.getCmdTimeout(); // get the stored command velocity timeout
  Serial.print("motor command vel timeout in ms: ");
  Serial.println(cmd_vel_timeout);

  sendHigh = true;

  readTime = millis();
  cmdTime = millis();
}

void loop()
{
  if ((millis() - cmdTime) >= cmdTimeInterval)
  {
    if (sendHigh)
    {
      v = vel;
      controller.writeSpeed(v, v);
      vel *= -1;
      sendHigh = false;
    }
    else
    {
      v = 0.0;
      controller.writeSpeed(v, v);
      sendHigh = true;
    }
    cmdTime = millis();
  }

  if ((millis() - readTime) >= readTimeInterval)
  {
    // controller.writeSpeed(v, v);
    controller.readMotorData(pos0, pos1, vel0, vel1);

    // Print results
    Serial.println("-----------------------------------");
    Serial.print("Motor 0: ");
    Serial.print(pos0, 2); Serial.print("\t"); Serial.println(vel0, 4);
    Serial.print("Motor 1: ");
    Serial.print(pos1, 2); Serial.print("\t"); Serial.println(vel1, 4);
    Serial.println("------------------------------------");
    
    // Serial.println();

    readTime = millis();
  }
}