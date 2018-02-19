# I2C_DMAC
Arduino Zero (SAMD21) based non-blocking I2C library using the Direct Memory Access Controller (DMAC).

### __Version__

- Version V1.0.0 Add Arduino MKR and SAMD51 support, plus multiple I2C instances 
- Version V1.0.0 Intial release

### __Arduino Compatibility__

- Arduino/Genuino Zero
- Arduino Zero Pro
- Arduino M0 Pro
- Arduino M0
- Arduino MKR Series
- Support for SAMD51 microcontrollers using Adafruit's Metro/Feather M4 core code

### __Installation__

After download simply un-zip the file and place the DMAC and I2C_DMAC directories in your ...Arduino/libraries folder. The Arduino folder is the one where your sketches are usually located.

### __Usage__

Simply include the I2C_DMAC.h file at the beginning of your sketch:

**_#include <I2C_DMAC.h>_**

The I2C_DMAC object is created (instantiated) automatically and the object can be called using the I2C prefix, for example:

**_I2C.begin();_**

The I2C_DMAC library's functions operate in the following way:

The "init" functions simply set up the DMAC prior to transfer, while the "read" and "write" functions do the actual transmission.

All the other read and write functions are just a combination of the these three base level operations.

The write functions allow for the transmission of the device address, plus the following options:

- Device Address -> Data -> Data Count (bytes)
- Device Address -> 8-bit Register Address
- Device Address -> 16-bit Register Address
- Device Address -> 8-bit Register Address -> 1 Byte Data
- Device Address -> 8-bit Register Addresss -> Data -> Data Count (bytes)
- Device Address -> 16-bit Register Address -> 1 Byte Data
- Device Address -> 16-bit Register Address -> Data -> Data Count (bytes)

The 8-bit register address is used to access most small I2C devices, such as sensors, while the 16-bit resgister address can be used to access I2C EEPROM devices.

The read functions allow for the transmission of the device address, plus the reception of the following options:

- Device Address -> 1 Data Byte
- Device Address -> Data -> Data Count (bytes)

Single bytes of data are handled by the library, meaning that you can simply enter constants as a single byte of data without having to allocate any memory. This is useful for configuring an I2C device.

A block of data can be a simple array and needs to be declared and "in scope" for the duration of the transfer. The block data size is limited to 255 bytes of data, (including the register address length). This limitation in imposed by the hardware.

Note that the I2C_DMAC doesn't use a ring buffer like the standard Wire library, it simply allows you to send and receive data from memory already allocated in your program. This also makes it more efficient as it isn't necessary to pull data off the ring buffer, the data is instead transfer directly to where you specify.

By default the DMAC uses channel 0 to write and 1 to read, but it's possible to select your DMAC channels of choice (0-11 on SAMD21 based boards and 0-31 on the SAMD51). It's also possible to set the priority level (0 lowest-3 highest). This is only necessary if you're using the DMAC channels for other purposes as well.

It's possible to initialise the DMAC only one time and then continuouly call the read() and write() functions in the loop() to initiate multiple transfers. In other words it isn't necessary to set-up the DMAC each time if you're doing a repeated operation.

To allow the sketch to check if the DMAC read or write operation is complete it's necessary to poll the respective busy flags:

**_while(I2C.writeBusy);_**

It's also possible to allocate callback functions that are executed when a read or write has completed, or when an error occurs.

The DMAC_Handler() and SERCOM3_Handler are provided as weak linker symbols allowing them to be overriden in your sketch for inclusion of your own handler functions, should that be necessary.

The latest version includes support for mulitple I2C instances provided the instances are assigned different DMAC channels. An demonstration sketch using two MPU6050 gyroscope/accelerometer devices is included in the example code.

### __Example Code__

The examples directory includes I2C_DMAC example code for the MPU6050 gyroscope/accelerometer device.
