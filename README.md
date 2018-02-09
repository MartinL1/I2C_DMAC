# I2C_DMAC
Arduino Zero (SAMD21) based non-blocking I2C library using the Direct Memory Access Controller (DMAC).

Installation

After download simply un-zip the file and place the DMAC and I2C_DMAC directories in your ...Arduino/libraries folder. The Arduino folder is the one where your sketches are usually located.

Usage

Simply include the I2C_DMAC.h file the beginning of the sketch:

#include <I2C_DMAC.h>

The I2C_DMAC object is created (instantiated) automatically and the object can be called using the I2C prefix, for example:

I2C.begin()

The examples directory includes I2C_DMAC example code for the MPU6050 gyroscope/accelerometer device.
