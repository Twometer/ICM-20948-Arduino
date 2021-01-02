# ICM-20948-Arduino
Arduino library for the ICM-20948 IMU with DMP support

Tested and works with ESP8266



## Using this library

To use this library, **the proprietary driver from Invensense is required**. According to their license, I cannot upload the code to GitHub. Therefore, this library is only a wrapper for their code and does not contain the driver itself.

Therefore, you have to log in to Invensense's [developer portal](https://invensense.tdk.com/developers/software-downloads/) and download `ICM-20948 eMD 1.0 for Nucleo Board`. Although we don't use the nucleo board, the C library is compatible. Navigate through the folder structure to find the `Invn` folder and copy it into the `src` folder of the Arduino library.

Then you can use the lib as normal.

