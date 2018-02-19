#include <I2C_DMAC.h>
#include <wiring_private.h>

#define MPU6050_ADDRESS 0x68                                // Device address when ADO = 0
#define PWR_MGMT_1      0x6B
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define GYRO_XOUT_H     0x43

uint8_t data[6];
uint8_t data1[6];
volatile boolean sercomError = false;

I2C_DMAC I2C1(&sercom2, 4, 3);                              // Create (instantiate) a second I2C1 object with sercom2, on pins D3 and D4

void setup() 
{
  SerialUSB.begin(115200);                                  // Activate the native USB port
  while(!SerialUSB);                                        // Wait for the native USB to be ready
  I2C.begin(400000);                                        // Start I2C bus at 400kHz          
  I2C.writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);         // Wake up the MPU6050 device
  delay(100);                                               // Wait for the MPU6050 to settle   
  I2C.writeByte(MPU6050_ADDRESS, CONFIG, 0x4);              // Set the gyro/accel filter to 20Hz  
  I2C.writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x1 << 3);    // Set full scale range to +/-500 degrees/s 
  I2C.writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x1 << 3);   // Set full scale range to +/-4g
  I2C.initWriteRegAddr(MPU6050_ADDRESS, GYRO_XOUT_H);       // Set-up DMAC to write to MPU6050 register pointer
  I2C.initReadBytes(MPU6050_ADDRESS, data, sizeof(data));   // Set DMAC to read the data
  I2C.attachReadCallback(gyro1Callback);                    // Attach a read callback function to I2C
  
  I2C1.begin(400000);                                       // Start I2C bus at 400kHz 
  pinPeripheral(3, PIO_SERCOM_ALT);                         // Assign D3 to SERCOM2 I2C SDA
  pinPeripheral(4, PIO_SERCOM_ALT);                         // Assign D4 to SERCOM2 I2C SCL
  I2C1.setWriteChannel(2);                                  // Set the I2C1 DMAC write channel to 2
  I2C1.setReadChannel(3);                                   // Set the I2C1 DMAC read channel to 3
  I2C1.writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);        // Wake up the MPU6050 device
  delay(100);                                               // Wait for the MPU6050 to settle   
  I2C1.writeByte(MPU6050_ADDRESS, CONFIG, 0x4);             // Set the gyro/accel filter to 20Hz  
  I2C1.writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x1 << 3);   // Set full scale range to +/-500 degrees/s  
  I2C1.writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x1 << 3);  // Set full scale range to +/-4g
  I2C1.initWriteRegAddr(MPU6050_ADDRESS, GYRO_XOUT_H);      // Set-up DMAC to write to MPU6050 register pointer
  I2C1.initReadBytes(MPU6050_ADDRESS, data1, sizeof(data1));// Set DMAC to read the data
  I2C1.attachReadCallback(gyro2Callback);                   // Attach a read callback function to I2C1
}

void loop() 
{ 
  I2C.write();                                              // DMAC write to set MPU6050 register pointer to start of the data
  // Add your concurrent code here...
  while(I2C.writeBusy);                                     // Wait for synchronization 
  I2C.read();
  // Add your concurrent code here...
  while(I2C.readBusy);                                      // Wait for synchronization 
  //SerialUSB.print(F("Gyro 1: "));                           // Acquire raw gyro values from MPU6050 device 1
  SerialUSB.print((int16_t)((data[0] << 8) | data[1]));     
  SerialUSB.print(F("   "));
  SerialUSB.print((int16_t)((data[2] << 8) | data[3]));
  SerialUSB.print(F("   "));
  SerialUSB.println((int16_t)((data[4] << 8) | data[5]));

  I2C1.write();                                              // DMAC write to set MPU6050 register pointer to start of the data
  // Add your concurrent code here...
  while(I2C1.writeBusy);                                     // Wait for synchronization 
  I2C1.read();
  // Add your concurrent code here...
  while(I2C1.readBusy);                                      // Wait for synchronization 
  //SerialUSB.print(F("Gyro 2: "));                            // Acquire raw gyro values from MPU6050 device 2
  SerialUSB.print((int16_t)((data1[0] << 8) | data1[1]));    
  SerialUSB.print(F("   "));
  SerialUSB.print((int16_t)((data1[2] << 8) | data1[3]));
  SerialUSB.print(F("   "));
  SerialUSB.println((int16_t)((data1[4] << 8) | data1[5]));
}

void gyro1Callback()
{
  SerialUSB.print(F("Gyro 1: "));
}

void gyro2Callback()
{
  SerialUSB.print(F("Gyro 2: "));
}



