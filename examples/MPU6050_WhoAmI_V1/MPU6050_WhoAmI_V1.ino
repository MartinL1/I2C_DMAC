#include <I2C_DMAC.h>

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

