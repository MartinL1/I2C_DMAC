#include <I2C_DMAC.h>

#define MPU6050_ADDRESS 0x68      // Device address when ADO = 0
#define PWR_MGMT_1      0x6B
#define TEMP_OUT_H      0x41

uint8_t data[2];

void setup() 
{
  SerialUSB.begin(115200);        // Activate the native USB port
  while(!SerialUSB);              // Wait for the native USB to be ready

  I2C.begin(400000);                                        // Start I2C bus at 400kHz
  //I2C.attachSercom3ErrorCallback(sercom3errorCallback);
  I2C.writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);         // Wake up the MPU6050 device
  delay(100);                                               // Wait for the MPU6050 to settle   
  I2C.initWriteRegAddr(MPU6050_ADDRESS, TEMP_OUT_H);        // Set-up DMAC to write to MPU6050 register pointer
  I2C.initReadBytes(MPU6050_ADDRESS, data, sizeof(data));   // Set-up DMAC to read the data
}

void loop() 
{ 
  I2C.write();                                // DMAC write to set MPU6050 register pointer to start of the data
  // Add your concurrent code here...
  while(I2C.writeBusy);                       // Wait for synchronization 
  I2C.read();
  // Add your concurrent code here...
  while(I2C.readBusy);                        // Wait for synchronization 
  SerialUSB.println((float)((int16_t)((data[0] << 8) | data[1])) / 340.0f + 36.53f);     // Acquire raw gyro values 
}




