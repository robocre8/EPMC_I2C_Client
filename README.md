## Easy PID Motor Controller (EPMC) Arduino I2C Client Library - EPMC_I2C_Client
This library allows Arduino-compatible boards to communicate with the **Easy PID Motor Controller (EPMC)** over I2C for motor control and feedback, after successful setup with the [epmc_setup_application](https://github.com/samuko-things-company/epmc_setup_application).

> you can use it in your Arduino-based robotics project (e.g Arduino UNO, Arduino NANO, Arduino MEGA, Esp32, etc.)

A simple way to get started is simply to try out and follow the example code

> NOTE: This library assumes IEEE-754 little-endian float representation on both the controller and MCU.


## How to Use the Library
- Ensure you have the **`Easy PID Motor Controller Module`**. Calibrate it and set it up using the **`epmc_setup_application`**.

- Download download the library by clicking on the green Code button above (or clone it)
  > if you download it, extract it and change the folder name to `EPMC_I2C_Client`

- Move the downloaded library file - **`EPMC_I2C_Client`** - to your Arduino library folder
  > e.g on linux: ... home/Arduino/libraries/
  >
  > e.g on windows: ... Documents/Arduino/libraries/
  >
  > (or any where your arduino libraries are stored)

- restart your ArduinoIDE and navigate to examples and run the example code and see how to control the motors.

- you can copy this example code into your project and modify it to your taste.