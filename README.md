# I2C_DMAC
Arduino Zero (SAMD21/SAMD51) based non-blocking I2C library using the Direct Memory Access Controller (DMAC). 

This library employs the SAMD21/SAMD51's DMAC module to transfer data to and from I2C sensors or EEPROM, while simultaneously freeing up the CPU to perform other calculations in the meantime. In time critical applications this can lead to a significant boost in processor performance. 

The library was originally designed to significantly increase the performance of the Arduino compatible, I2C sensor based Falcon 1 and 2 mutli-rotor (drone) flight controllers:

<img src="https://static.rcgroups.net/forums/attachments/6/0/1/5/8/7/a9606835-172-FalconDiagSmall.png" alt="Falcon Flight Controllers" width="320" height="240"><img src="https://static.rcgroups.net/forums/attachments/6/0/1/5/8/7/a9606837-83-Falcon2Setup.png" alt="Falcon 2 Flight Controller" width="320" height="240">

## __Version__

- Version V1.1.6 -- Add SERCOM ALT (alternative) peripheral switch for the Metro M4
- Version V1.1.5 -- Activate internal pull-up resistors and increase driver strength
- Version V1.1.4 -- Allow the DMAC to resume normal operation after an early NACK is received
- Version V1.1.3 -- Fixed issue with consecutive calls to writeByte() overwriting data
- Version V1.1.2 -- Allow other classes to simultaneously use remaining DMAC channels
- Version V1.1.1 -- Replaced pinPeripheral() function with port register manipulation
- Version V1.1.0 -- Add Arduino MKR and SAMD51 support, plus multiple I2C instances 
- Version V1.0.0 -- Intial release

## __Arduino Compatibility__

- Arduino/Genuino Zero
- Arduino Zero Pro
- Arduino M0 Pro
- Arduino M0
- Arduino MKR Series
- Support for SAMD51 microcontrollers using Adafruit's Metro/Feather M4 core code

## __Installation__

After download simply un-zip the file and place the I2C_DMAC directory in your ...Arduino/libraries folder. The Arduino folder is the one where your sketches are usually located.

## __Usage__

### __I2C_DMAC Library__

Simply include the I2C_DMAC.h file at the beginning of your sketch:

```
#include <I2C_DMAC.h>
```
---

### __Initialisation__

The I2C_DMAC object is created (instantiated) automatically and the object can be called using the I2C prefix, for example:

```
I2C.begin();
```

Note that for the Metro M4 board it is necessary to specify that the I2C port is using the alternative SERCOM (SERCOM_ALT), as well as specifying either an 8-bit or 16-bit register address:

```
I2C.begin(400000, REG_ADDR_8BIT, PIO_SERCOM_ALT);
```
---

### __Reading and Writing__

The I2C_DMAC library's functions operate in the following way:

The "init" functions simply set up the DMAC prior to transfer, while the "read" and "write" functions do the actual transmission:



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

Single bytes of data are handled by the library, meaning that you can simply enter constants as a single byte of data without having to allocate any memory. This is useful for configuring an I2C device or reading single 8-bit registers:

```
I2C.readByte(MPU6050_ADDRESS, WHO_AM_I);           // Read the WHO_AM_I register 
while(I2C.readBusy);                               // Wait for synchronization
SerialUSB.println(I2C.getData(), HEX);             // Output the result
```

A block of data can be a simple array and needs to be declared and "in scope" for the duration of the transfer. The block data size is limited to 255 bytes of data, (including the register address length). This limitation in imposed by the hardware:

```
I2C.readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, data, 6);      // Read the data registers 
while(I2C.readBusy);                                       // Wait for synchronization
SerialUSB.print((int16_t)((data[0] << 8) | data[1]));      // Display the raw gyroscope values
SerialUSB.print(F("   "));
SerialUSB.print((int16_t)((data[2] << 8) | data[3]));
SerialUSB.print(F("   "));
SerialUSB.println((int16_t)((data[4] << 8) | data[5]));
```

Note that the I2C_DMAC doesn't use a ring buffer like the standard Wire library, it simply allows you to send and receive data from memory already allocated in your program. This also makes it more efficient as it isn't necessary to pull data off the ring buffer, the data is instead transfer directly to where you specify.

By default the DMAC uses channel 0 to write and 1 to read, but it's possible to select your DMAC channels of choice (0-11 on SAMD21 based boards and 0-31 on the SAMD51). It's also possible to set the priority level (0 lowest-3 highest). This is only necessary if you're using the DMAC channels for other purposes as well.

It's possible to initialise the DMAC only one time and then continuouly call the read() and write() functions in the loop() to initiate multiple transfers. In other words it isn't necessary to set-up the DMAC each time if you're doing a repeated operation.

To allow the sketch to check if the DMAC read or write operation is complete it's necessary to poll the respective busy flags:

```
while(I2C.writeBusy);
```

It's also possible to allocate callback functions that are executed when a read or write has completed, or when an error occurs.

The DMAC_Handler() and SERCOM3_Handler are provided as weak linker symbols allowing them to be overriden in your sketch for inclusion of your own handler functions, should that be necessary.

The latest version includes support for mulitple I2C instances, provided the instances are assigned different DMAC channels. A demonstration sketch: "MPU6050_Gyroscope_V2.ino", using two MPU6050 gyroscope/accelerometer devices is included in the example code.

---

### __Code Implementation__

An example of accessing the WHO_AM_I register on the MPU6050 gyroscope/accelerometer using the three levels of functions provided by the I2C_DMAC library

```#include <I2C_DMAC.h>

#define MPU6050_ADDRESS 0x68                         // Device address when ADO = 0
#define WHO_AM_I        0x75                         // Should return 0x68

void setup() 
{
  SerialUSB.begin(115200);                           // Activate the native USB port
  while(!SerialUSB);                                 // Wait for the native USB to be ready

  // Combined DMAC initialisation, write and read
  I2C.begin(400000);                                 // Start I2C bus at 400kHz
  I2C.readByte(MPU6050_ADDRESS, WHO_AM_I);           // Read the WHO_AM_I register 
  while(I2C.readBusy);                               // Wait for synchronization  
  SerialUSB.println(I2C.getData(), HEX);             // Output the result

  // Or combined DMAC and write initialisation, followed by read
  I2C.writeRegAddr(MPU6050_ADDRESS, WHO_AM_I);       // Write the register address
  I2C.readByte(MPU6050_ADDRESS);                     // Read the WHO_AM_I register
  while(I2C.readBusy);                               // Wait for synchronization
  SerialUSB.println(I2C.getData(), HEX);             // Output the result

  // Or indpendent DMAC initialisation, write and read
  I2C.initWriteRegAddr(MPU6050_ADDRESS, WHO_AM_I);   // Initialise the DMAC write
  I2C.initReadByte(MPU6050_ADDRESS);                 // Initialise the DMAC read
  I2C.write();                                       // Write the register address
  while(I2C.writeBusy);                              // Wait for synchronization
  I2C.read();                                        // Read the WHO_AM_I register
  while(I2C.readBusy);                               // Wait for synchronization
  SerialUSB.println(I2C.getData(), HEX);             // Output the result 
}

void loop() {}
```

---

### __Example Code__

The examples directory includes I2C_DMAC example code for the MPU6050 gyroscope/accelerometer device. 

(Note that Adafruit boards use Serial rather than SerialUSB for console communications).
